// MANUS glove to ZMQ bridge.
//
// Attaches directly to the MANUS SDK in Core Integrated mode, receives the raw
// skeleton stream and publishes every frame over ZMQ. The Python retargeting
// side subscribes to it.
//
// This is the only C++ in the project that depends on the SDK. What leaves here
// is a plain binary frame, so nothing below this has any MANUS dependency.
//
// On haptic input:
//   MANUS Core Integrated allows **only one instance system-wide** to hold the
//   SDK. Measured: while the bridge is running, another process that tries to
//   connect gets a "Make sure to shut down all other instances of Core
//   Integrated" warning, receives no data, and takes the existing bridge stream
//   down with it. Vibration commands therefore have to pass through this
//   process, which owns the SDK.
//
//   In exchange, **without --haptics the entire path is dead** -- no socket is
//   opened and no thread starts. That is deliberate, so it cannot affect the
//   teleoperation stream.
//
//   **Haptics also gets one socket per hand** (left 5556, right 5558).
//   There is one reason: **to let the socket identify the hand**. Previously,
//   when a node sent glove_id 0 the bridge read it as "left first", so with both
//   gloves connected, starting the right-hand node sent **all vibration to the
//   left glove**. Avoiding that meant the user had to read the hexadecimal ID
//   off the status line and pass it with --glove-id. Splitting the socket
//   removes that knob entirely.
//
//   NOTE: the reason differs from the stream side (PUB). There, CONFLATE mixed
//   the two hands together, which is why those were split. Here that problem
//   does not exist -- **ZMQ's CONFLATE is per peer (pipe), not per socket**, so
//   two senders each keep their own newest value (measured: forcing loss by
//   sending 300 messages each through a single socket still gave an even
//   left 31 / right 30). With only one glove connected, either socket reaches
//   it (see SideGloveId below).
//
// On the transport:
//   PUB/SUB is used. PUSH/PULL round-robins frames between multiple consumers,
//   so attaching a simulator later would steal half the frames from retargeting.
//   PUB gives every subscriber the same frame.
//   CONFLATE=1 on both ends keeps the queue from backing up, always leaving
//   only the newest frame.
//
//   **One socket per hand** (left 5555, right 5557). CONFLATE keeps only the
//   newest entry in the queue, so sending both hands through one socket lets
//   their frames overwrite each other and the receiver gets an interleaved
//   stream. Filtering by side at the receiver cannot fix it -- by the time you
//   could filter, the other hand's frame is already gone.
//
// Build:  from the repository root,  make bridge
//         (if the SDK is unpacked elsewhere: make bridge MANUS_SDK=/path/ManusSDK)
//         WARNING: always rebuild after editing this file. A stale binary still
//         looks like it is working, which makes it hard to spot.
//
// Run:
//   ./manus_bridge                                  left 5555 / right 5557
//   ./manus_bridge tcp://127.0.0.1:5555 --right tcp://127.0.0.1:5557
//   ./manus_bridge --haptics tcp://127.0.0.1:5556   also opens vibration input
//                                                   (left 5556 / right 5558)
//   ./manus_bridge --haptics tcp://127.0.0.1:5556 --haptics-right tcp://127.0.0.1:5558

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"

#include <zmq.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

// ── Wire format ─────────────────────────────────────────────────────────
// Little-endian, fixed layout. The Python side reads it straight with numpy.
//
//   uint32  magic      = 'MANU' (0x554E414D)
//   uint32  version    = 1
//   uint64  t_ns                 send time (CLOCK_MONOTONIC, ns)
//   uint32  glove_id
//   uint32  side                 1=left, 2=right
//   uint32  node_count           = 25
//   float32 nodes[node_count][7] (px,py,pz, qx,qy,qz,qw)
//
// Always bump the version field after magic when the format changes -- failing
// loudly beats a subscriber silently misreading the data.
static constexpr uint32_t kMagic = 0x554E414DU;
static constexpr uint32_t kVersion = 1;
static constexpr uint32_t kHeaderBytes = 4 + 4 + 8 + 4 + 4 + 4;
static constexpr uint32_t kFloatsPerNode = 7;

static std::atomic<bool> g_stop{false};
static void OnSigInt(int) { g_stop = true; }

static void* g_zmq_ctx = nullptr;
// One socket per hand. With CONFLATE=1 on a single socket, sending both hands
// through it lets their frames overwrite each other and the receiver gets an
// interleaved stream.
static void* g_pub_left  = nullptr;
static void* g_pub_right = nullptr;

// ── Haptics (optional feature) ──────────────────────────────────────────
// Wire format: uint32 magic 'MHAP' + uint32 glove_id + float32[5] powers
//   powers order = Thumb, Index, Middle, Ring, Pinky (SDK convention), 0.0-1.0
// A glove_id of 0 means **the hand this socket serves**. If that hand is not
// connected it falls through to the other one -- a convenience for single-glove
// use.
static constexpr uint32_t kHapticMagic = 0x5041484DU;  // 'MHAP'
static constexpr size_t kHapticBytes = 4 + 4 + 5 * sizeof(float);

static std::atomic<bool> g_haptics_enabled{false};
static std::atomic<uint64_t> g_haptic_cmds{0};
static std::atomic<uint64_t> g_haptic_errors{0};
static std::thread g_haptic_threads[2];   // [0]=left [1]=right

// Loop that receives vibration commands and hands them to the SDK. It runs on
// a thread **completely separate from the skeleton callback**, so
// RawSkeletonInfo carries no left/right information. The landscape stream gives
// us the glove ID to side mapping, which we cache and attach to each frame.
static std::atomic<uint32_t> g_left_id{0};
static std::atomic<uint32_t> g_right_id{0};

static void OnLandscape(const Landscape* p_Landscape)
{
    const auto& gloves = p_Landscape->gloveDevices;
    for (uint32_t i = 0; i < gloves.gloveCount; i++)
    {
        const GloveLandscapeData& g = gloves.gloves[i];
        if (g.side == Side_Left) g_left_id = g.id;
        if (g.side == Side_Right) g_right_id = g.id;
    }
}

// Frames whose side is not yet identified (no landscape received) are
// **discarded**. Publishing one without knowing the hand could make the other
// hand follow it. The discard count prints on the status line every second, so
// a failure to identify shows up immediately.
static void* PubForSide(uint32_t side)
{
    if (side == (uint32_t)Side_Left)  return g_pub_left;
    if (side == (uint32_t)Side_Right) return g_pub_right;
    return nullptr;
}

static uint32_t SideOf(uint32_t glove_id)
{
    if (glove_id != 0 && glove_id == g_left_id.load()) return (uint32_t)Side_Left;
    if (glove_id != 0 && glove_id == g_right_id.load()) return (uint32_t)Side_Right;
    return (uint32_t)Side_Invalid;
}

// The glove ID of the hand this socket serves, falling through to the other
// hand when it is absent (so either socket works with a single glove).
// With both gloves connected, each socket always reaches its own hand.
static uint32_t SideGloveId(uint32_t side)
{
    const uint32_t own   = (side == (uint32_t)Side_Right) ? g_right_id.load() : g_left_id.load();
    if (own != 0) return own;
    const uint32_t other = (side == (uint32_t)Side_Right) ? g_left_id.load()  : g_right_id.load();
    return other;
}

// Even if this stalls or blocks, the 120 Hz stream is unaffected.
// side is the hand this socket serves (Side_Left / Side_Right).
static void HapticLoop(void* sub, uint32_t side)
{
    uint8_t buf[64];
    while (!g_stop)
    {
        const int n = zmq_recv(sub, buf, sizeof(buf), ZMQ_DONTWAIT);
        if (n < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if ((size_t)n != kHapticBytes)
        {
            g_haptic_errors++;
            continue;
        }
        uint32_t magic = 0, glove_id = 0;
        std::memcpy(&magic, buf, 4);
        std::memcpy(&glove_id, buf + 4, 4);
        if (magic != kHapticMagic)
        {
            g_haptic_errors++;
            continue;
        }
        float powers[5];
        std::memcpy(powers, buf + 8, sizeof(powers));
        for (float& v : powers)
            v = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);   // range guard

        if (glove_id == 0)
            glove_id = SideGloveId(side);
        if (glove_id == 0)
        {
            g_haptic_errors++;
            continue;
        }

        // Count failures quietly; never kill the bridge over one.
        if (CoreSdk_VibrateFingersForGlove(glove_id, powers) != SDKReturnCode_Success)
            g_haptic_errors++;
        else
            g_haptic_cmds++;
    }

    // Always turn vibration off on exit, or the last strength stays latched.
    // Each thread clears only its own hand (with both hands there are two).
    const float off[5] = {0, 0, 0, 0, 0};
    const uint32_t id = SideGloveId(side);
    if (id != 0) CoreSdk_VibrateFingersForGlove(id, off);
}


static std::atomic<uint64_t> g_frames{0};
// Per-hand counters. Printing only the total shows something like "226 Hz"
// with both hands, which hides the real per-hand frame rate.
static std::atomic<uint64_t> g_frames_left{0};
static std::atomic<uint64_t> g_frames_right{0};
static std::atomic<uint64_t> g_dropped{0};
static std::atomic<uint64_t> g_unidentified{0};   // frames dropped, side unknown
static std::atomic<uint32_t> g_node_count{0};
static std::atomic<uint64_t> g_last_frame_ns{0};
static std::atomic<uint64_t> g_recoveries{0};

// After a few idle minutes the glove disconnects silently at the SDK level (USB
// stays up). The log shows only "Glove is disconnected" and the skeleton
// callback stops. If the bridge did nothing, subscribers would see a hand frozen
// in its last posture -- dangerous on real hardware.
// If frames stop for longer than this, restart the SDK and attempt recovery.
static constexpr uint64_t kStaleNs = 3000ULL * 1000000ULL;

// After an SDK restart it takes a few seconds for the glove to reattach and the
// stream to resume. Without this grace period the sequence "detect stall ->
// restart -> still no frames -> restart again" runs away and never recovers.
static constexpr uint64_t kGraceNs = 15000ULL * 1000000ULL;

static uint64_t NowNs()
{
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void OnSkeleton(const SkeletonStreamInfo* p_Info)
{
    for (uint32_t i = 0; i < p_Info->skeletonsCount; i++)
    {
        RawSkeletonInfo skel;
        if (CoreSdk_GetRawSkeletonInfo(i, &skel) != SDKReturnCode_Success)
            continue;
        if (skel.nodesCount == 0)
            continue;

        std::vector<SkeletonNode> nodes(skel.nodesCount);
        if (CoreSdk_GetRawSkeletonData(i, nodes.data(), skel.nodesCount) != SDKReturnCode_Success)
            continue;

        const uint32_t side = SideOf(skel.gloveId);
        void* pub = PubForSide(side);
        if (pub == nullptr)
        {
            g_unidentified++;
            continue;
        }

        g_node_count = skel.nodesCount;
        g_last_frame_ns = NowNs();

        const size_t payload = kHeaderBytes + (size_t)skel.nodesCount * kFloatsPerNode * sizeof(float);
        std::vector<uint8_t> buf(payload);
        uint8_t* p = buf.data();

        auto put_u32 = [&p](uint32_t v) { std::memcpy(p, &v, 4); p += 4; };
        auto put_u64 = [&p](uint64_t v) { std::memcpy(p, &v, 8); p += 8; };

        put_u32(kMagic);
        put_u32(kVersion);
        put_u64(NowNs());
        put_u32(skel.gloveId);
        put_u32(side);
        put_u32(skel.nodesCount);

        float* f = reinterpret_cast<float*>(p);
        for (uint32_t n = 0; n < skel.nodesCount; n++)
        {
            const ManusTransform& t = nodes[n].transform;
            *f++ = t.position.x; *f++ = t.position.y; *f++ = t.position.z;
            *f++ = t.rotation.x; *f++ = t.rotation.y; *f++ = t.rotation.z; *f++ = t.rotation.w;
        }

        // ZMQ_DONTWAIT: never block the SDK callback thread on a slow subscriber.
        // Blocking here would back up the entire glove stream. Frames we could
        // not send are counted and reported.
        if (zmq_send(pub, buf.data(), buf.size(), ZMQ_DONTWAIT) < 0)
            g_dropped++;
        else
        {
            g_frames++;
            if (side == (uint32_t)Side_Right) g_frames_right++;
            else                              g_frames_left++;
        }
    }
}

// Bring the SDK up from scratch. Recovery repeats exactly the same steps.
static bool StartSdk()
{
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success)
    { printf("[bridge] InitializeIntegrated failed\n"); return false; }

    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape);
    CoreSdk_RegisterCallbackForRawSkeletonStream(OnSkeleton);

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up = AxisPolarity_PositiveZ;
    vuh.view = AxisView_XFromViewer;
    vuh.unitScale = 1.0f;
    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success)
    { printf("[bridge] coordinate system setup failed\n"); return false; }

    if (CoreSdk_LookForHosts(1, false) != SDKReturnCode_Success)
    { printf("[bridge] LookForHosts failed\n"); return false; }
    uint32_t hosts = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&hosts);
    if (hosts == 0) { printf("[bridge] no host found\n"); return false; }

    std::vector<ManusHost> host_list(hosts);
    CoreSdk_GetAvailableHostsFound(host_list.data(), hosts);
    if (CoreSdk_ConnectToHost(host_list[0]) == SDKReturnCode_NotConnected)
    { printf("[bridge] ConnectToHost failed\n"); return false; }

    // SDK 3.1.1: must be called after connecting (calling it before segfaults).
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    return true;
}

static void StopSdk()
{
    CoreSdk_RegisterCallbackForRawSkeletonStream(nullptr);
    CoreSdk_RegisterCallbackForLandscapeStream(nullptr);
    CoreSdk_ShutDown();
}

int main(int argc, char** argv)
{
    const char* endpoint_left  = "tcp://127.0.0.1:5555";
    const char* endpoint_right = "tcp://127.0.0.1:5557";
    const char* haptic_left  = nullptr;
    const char* haptic_right = "tcp://127.0.0.1:5558";
    for (int i = 1; i < argc; i++)
    {
        if (std::strcmp(argv[i], "--haptics") == 0 && i + 1 < argc)
            haptic_left = argv[++i];
        else if (std::strcmp(argv[i], "--haptics-right") == 0 && i + 1 < argc)
            haptic_right = argv[++i];
        else if (std::strcmp(argv[i], "--right") == 0 && i + 1 < argc)
            endpoint_right = argv[++i];
        else if (argv[i][0] != '-')
            endpoint_left = argv[i];
    }

    signal(SIGINT, OnSigInt);
    signal(SIGTERM, OnSigInt);

    g_zmq_ctx = zmq_ctx_new();
    int conflate = 1, linger = 0;
    struct { void** sock; const char* ep; const char* name; } pubs[] = {
        { &g_pub_left,  endpoint_left,  "left"  },
        { &g_pub_right, endpoint_right, "right" },
    };
    for (auto& p : pubs)
    {
        *p.sock = zmq_socket(g_zmq_ctx, ZMQ_PUB);
        zmq_setsockopt(*p.sock, ZMQ_CONFLATE, &conflate, sizeof(conflate));
        zmq_setsockopt(*p.sock, ZMQ_LINGER, &linger, sizeof(linger));
        if (zmq_bind(*p.sock, p.ep) != 0)
        {
            printf("[bridge] FAILED zmq_bind(%s): %s\n", p.ep, zmq_strerror(zmq_errno()));
            return 1;
        }
        printf("[bridge] publishing %-5s on %s\n", p.name, p.ep);
    }

    // Without --haptics nothing below happens (no socket, no thread).
    // With it, one socket per hand is opened; --haptics-right changes the
    // right-hand address.
    void* haptic_subs[2] = { nullptr, nullptr };
    if (haptic_left != nullptr)
    {
        struct { const char* ep; const char* name; } haps[2] = {
            { haptic_left,  "left"  },
            { haptic_right, "right" },
        };
        for (int i = 0; i < 2; i++)
        {
            void* sub = zmq_socket(g_zmq_ctx, ZMQ_PULL);
            zmq_setsockopt(sub, ZMQ_CONFLATE, &conflate, sizeof(conflate));
            zmq_setsockopt(sub, ZMQ_LINGER, &linger, sizeof(linger));
            if (zmq_bind(sub, haps[i].ep) != 0)
            {
                printf("[bridge] haptic socket bind failed (%s %s): %s -- continuing without haptics for this hand\n",
                       haps[i].name, haps[i].ep, zmq_strerror(zmq_errno()));
                zmq_close(sub);
                continue;
            }
            haptic_subs[i] = sub;
            g_haptics_enabled = true;
            printf("[bridge] haptic input %-5s %s\n", haps[i].name, haps[i].ep);
        }
    }

    if (!StartSdk())
        return 1;
    g_last_frame_ns = NowNs();

    for (int i = 0; i < 2; i++)
        if (haptic_subs[i] != nullptr)
            g_haptic_threads[i] = std::thread(
                HapticLoop, haptic_subs[i],
                i == 0 ? (uint32_t)Side_Left : (uint32_t)Side_Right);

    printf("[bridge] running -- Ctrl+C to stop\n");
    uint64_t last_frames = 0, last_left = 0, last_right = 0;
    bool stale = false;
    uint64_t grace_until = NowNs() + kGraceNs;

    while (!g_stop)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t now = NowNs();
        const uint64_t f = g_frames.load();

        // Print this even when no glove is attached and the stream is down --
        // it must be possible to check the haptic socket without a glove.
        if (g_haptics_enabled.load())
        {
            static uint64_t last_haptic = ~0ULL;
            const uint64_t hc = g_haptic_cmds.load();
            const uint64_t he = g_haptic_errors.load();
            if (hc + he != last_haptic)
            {
                printf("[bridge]   haptic received %lu / errors %lu\n",
                       (unsigned long)hc, (unsigned long)he);
                last_haptic = hc + he;
            }
        }
        const bool advancing = (f > last_frames);
        const uint64_t age = now - g_last_frame_ns.load();

        if (advancing)
        {
            if (stale)
            {
                stale = false;
                printf("[bridge] recovered -- stream resumed\n");
            }
            // Print Hz per hand, showing only the hands that are attached.
            const uint64_t fl = g_frames_left.load(), fr = g_frames_right.load();
            char rate[64];
            if (g_left_id.load() != 0 && g_right_id.load() != 0)
                snprintf(rate, sizeof(rate), "left %3u / right %3u Hz",
                         (unsigned)(fl - last_left), (unsigned)(fr - last_right));
            else if (g_right_id.load() != 0)
                snprintf(rate, sizeof(rate), "right %3u Hz", (unsigned)(fr - last_right));
            else
                snprintf(rate, sizeof(rate), "left %3u Hz", (unsigned)(fl - last_left));
            last_left = fl; last_right = fr;

            printf("[bridge] %-22s | nodes=%u | %lu frames | undelivered %lu | "
                   "unidentified %lu | %lu recoveries | gloves L=0x%X R=0x%X\n",
                   rate, g_node_count.load(),
                   (unsigned long)f, (unsigned long)g_dropped.load(),
                   (unsigned long)g_unidentified.load(),
                   (unsigned long)g_recoveries.load(),
                   g_left_id.load(), g_right_id.load());
            fflush(stdout);
            last_frames = f;
            continue;
        }

        // The frame count is not advancing. If we are inside the grace period,
        // keep waiting.
        if (now < grace_until)
        {
            printf("[bridge] waiting for stream (%.0f s left)\n", (grace_until - now) / 1e9);
            fflush(stdout);
            continue;
        }

        if (age > kStaleNs)
        {
            if (!stale)
            {
                stale = true;
                printf("[bridge] stream stalled (%.1f s) -- restarting SDK\n", age / 1e9);
            }
            StopSdk();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (StartSdk())
            {
                g_recoveries++;
                printf("[bridge] SDK restart #%lu -- waiting up to %.0f s\n",
                       (unsigned long)g_recoveries.load(), kGraceNs / 1e9);
            }
            else
            {
                printf("[bridge] restart failed -- will retry next cycle\n");
            }
            g_last_frame_ns = NowNs();
            grace_until = NowNs() + kGraceNs;
            fflush(stdout);
        }
    }

    printf("[bridge] shutting down...\n");
    for (int i = 0; i < 2; i++)
    {
        if (g_haptic_threads[i].joinable())
            g_haptic_threads[i].join();
        if (haptic_subs[i] != nullptr)
            zmq_close(haptic_subs[i]);
    }
    StopSdk();
    zmq_close(g_pub_left);
    zmq_close(g_pub_right);
    zmq_ctx_destroy(g_zmq_ctx);
    return 0;
}
