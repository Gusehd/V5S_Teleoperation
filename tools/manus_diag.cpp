// MANUS SDK diagnostic tool -- shows which data streams are actually flowing.
// Connects directly to the dongle in Core Integrated mode and periodically
// prints the event count and first node data for the landscape, ergonomics and
// raw skeleton streams.
//
// Build:
//   g++ -std=c++17 -I$SDK/include manus_diag.cpp -L$SDK/lib -lManusSDK_Integrated \
//       -Wl,-rpath,$SDK/lib -lpthread -o manus_diag

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <string_view>
#include <vector>
#include <cmath>

static std::atomic<bool> g_stop{false};
static std::atomic<uint64_t> g_landscape_events{0};
static std::atomic<uint64_t> g_ergonomics_events{0};
static std::atomic<uint64_t> g_skeleton_events{0};
static std::atomic<uint32_t> g_glove_count{0};
static std::atomic<uint32_t> g_left_id{0};
static std::atomic<uint32_t> g_right_id{0};

static std::mutex g_skel_mutex;
static uint32_t g_last_skel_count = 0;
static uint32_t g_last_glove_id = 0;
static uint32_t g_last_node_count = 0;
static ManusVec3 g_last_node1_pos{};

// Motion probe: tracks the maximum displacement of every node from its first
// sample. Watching node1 alone is misleading -- it sits near the wrist and may
// barely move even while the fingers do.
static std::vector<ManusVec3> g_motion_ref;
static std::vector<ManusQuaternion> g_rot_ref;
static float g_motion_max = 0.0f;
static float g_rot_max = 0.0f;

static std::mutex g_glove_detail_mutex;
static char g_glove_detail[1024] = "";

static std::atomic<uint32_t> g_user_count{0};

// Glove model identification (Quantum vs Metaglove Pro / Pro Haptic, etc.).
// The SDK already fills familyType and isHaptics in GloveLandscapeData, but the
// earlier code never read them -- added to diagnose which glove is actually
// connected.
static const char* FamilyTypeName(DeviceFamilyType t)
{
    switch (t)
    {
        case DeviceFamilyType_Unknown: return "Unknown";
        case DeviceFamilyType_Prime1: return "Prime1";
        case DeviceFamilyType_Prime2: return "Prime2";
        case DeviceFamilyType_PrimeX: return "PrimeX";
        case DeviceFamilyType_Metaglove: return "Metaglove(Quantum)";
        case DeviceFamilyType_Prime3: return "Prime3";
        case DeviceFamilyType_Virtual: return "Virtual";
        case DeviceFamilyType_MetaglovePro: return "MetaglovePro";
        case DeviceFamilyType_MetagloveProPrecision: return "MetagloveProPrecision";
        case DeviceFamilyType_MetagloveProHaptics: return "MetagloveProHaptics";
        case DeviceFamilyType_MetagloveProPrecisionHaptics: return "MetagloveProPrecisionHaptics";
        default: return "?";
    }
}

// Dongle license tier. Added so that a missing license can be diagnosed
// separately from excluded=1, in case METAGLOVES PRO HAPTIC requires a higher
// tier (Core 3 Plus or similar).
static const char* LicenseTypeName(LicenseType t)
{
    switch (t)
    {
        case LicenseType_Undefined: return "Undefined";
        case LicenseType_Polygon: return "Polygon";
        case LicenseType_CoreXO: return "CoreXO";
        case LicenseType_CorePro: return "CorePro";
        case LicenseType_CoreXOPro: return "CoreXOPro";
        case LicenseType_CoreX: return "CoreX";
        case LicenseType_CoreO: return "CoreO";
        case LicenseType_CoreQ: return "CoreQ";
        case LicenseType_CoreXPro: return "CoreXPro";
        case LicenseType_CoreOPro: return "CoreOPro";
        case LicenseType_CoreQPro: return "CoreQPro";
        case LicenseType_CoreXOQPro: return "CoreXOQPro";
        case LicenseType_CoreXR: return "CoreXR";
        default: return "?";
    }
}

static void OnLandscape(const Landscape* landscape)
{
    g_landscape_events++;
    const auto& gloves = landscape->gloveDevices;
    g_glove_count = gloves.gloveCount;
    g_user_count = landscape->users.userCount;
    char buf[1024];
    int off = 0;
    for (uint32_t i = 0; i < gloves.dongleCount && off < (int)sizeof(buf); i++)
    {
        const DongleLandscapeData& d = gloves.dongles[i];
        off += snprintf(buf + off, sizeof(buf) - off,
                        "  [dongle] netId=%u license=%s(%s) maxPairs=%u\n",
                        d.netDeviceID, d.licenseType, LicenseTypeName(d.licenseLevel),
                        d.licenseMaxNumberOfGlovePairs);
    }
    for (uint32_t i = 0; i < gloves.gloveCount; i++)
    {
        const GloveLandscapeData& g = gloves.gloves[i];
        if (g.side == Side::Side_Left) g_left_id = g.id;
        if (g.side == Side::Side_Right) g_right_id = g.id;
        off += snprintf(buf + off, sizeof(buf) - off,
                        "  [%s] id=0x%X family=%s haptics=%d paired=%d batt=%u%% rssi=%d excluded=%d imu(mag/acc/gyr/sys):",
                        g.side == Side::Side_Left ? "L" : (g.side == Side::Side_Right ? "R" : "?"),
                        g.id, FamilyTypeName(g.familyType), (int)g.isHaptics, (int)g.pairedState,
                        g.batteryPercentage, g.transmissionStrength, (int)g.excluded);
        for (int k = 0; k < MAX_NUM_IMUS_ON_GLOVE && off < (int)sizeof(buf); k++)
        {
            const IMUCalibrationInfo& c = g.iMUCalibrationInfo[k];
            off += snprintf(buf + off, sizeof(buf) - off, " %u/%u/%u/%u", c.mag, c.acc, c.gyr, c.sys);
        }
        off += snprintf(buf + off, sizeof(buf) - off, "\n");
    }
    for (uint32_t i = 0; i < landscape->users.userCount && off < (int)sizeof(buf); i++)
    {
        const UserLandscapeData& u = landscape->users.users[i];
        off += snprintf(buf + off, sizeof(buf) - off,
                        "  [user] id=%u name='%s' dongle=0x%X L=0x%X R=0x%X\n",
                        u.id, u.name, u.dongleID, u.leftGloveID, u.rightGloveID);
    }
    std::lock_guard<std::mutex> lock(g_glove_detail_mutex);
    snprintf(g_glove_detail, sizeof(g_glove_detail), "%s", buf);
}

static std::mutex g_ergo_mutex;
static float g_ergo_sample[6] = {0};
static uint32_t g_ergo_glove = 0;

static float g_ergo_min[20], g_ergo_max[20];
static bool g_ergo_seen = false;

static void OnErgonomics(const ErgonomicsStream* stream)
{
    g_ergonomics_events++;
    std::lock_guard<std::mutex> lock(g_ergo_mutex);
    for (uint32_t i = 0; i < stream->dataCount; i++)
    {
        if (stream->data[i].isUserID)
            continue;
        // Sample whichever side is connected. The original code was right-hand only.
        const uint32_t id = stream->data[i].id;
        if (id == 0 || (g_left_id != 0 && id != g_left_id && g_right_id != 0 && id != g_right_id))
            continue;
        g_ergo_glove = id;
        const bool is_left = (id == g_left_id);
        const int base = is_left ? ErgonomicsDataType_LeftFingerThumbMCPSpread
                                 : ErgonomicsDataType_RightFingerThumbMCPSpread;
        for (int k = 0; k < 20; k++)
        {
            const float v = stream->data[i].data[base + k];
            if (!g_ergo_seen) { g_ergo_min[k] = g_ergo_max[k] = v; }
            else { if (v < g_ergo_min[k]) g_ergo_min[k] = v; if (v > g_ergo_max[k]) g_ergo_max[k] = v; }
        }
        for (int k = 0; k < 6; k++)
            g_ergo_sample[k] = stream->data[i].data[base + k];
        g_ergo_seen = true;
    }
}

static void OnSkeleton(const SkeletonStreamInfo* info)
{
    g_skeleton_events++;
    std::lock_guard<std::mutex> lock(g_skel_mutex);
    g_last_skel_count = info->skeletonsCount;
    for (uint32_t i = 0; i < info->skeletonsCount; i++)
    {
        RawSkeletonInfo skel_info;
        if (CoreSdk_GetRawSkeletonInfo(i, &skel_info) != SDKReturnCode::SDKReturnCode_Success)
            continue;
        g_last_glove_id = skel_info.gloveId;
        g_last_node_count = skel_info.nodesCount;
        if (skel_info.nodesCount > 1)
        {
            std::vector<SkeletonNode> nodes(skel_info.nodesCount);
            if (CoreSdk_GetRawSkeletonData(i, nodes.data(), skel_info.nodesCount) ==
                SDKReturnCode::SDKReturnCode_Success)
            {
                g_last_node1_pos = nodes[1].transform.position;

                if (g_motion_ref.size() != nodes.size())
                {
                    g_motion_ref.resize(nodes.size());
                    for (size_t n = 0; n < nodes.size(); n++)
                        g_motion_ref[n] = nodes[n].transform.position;
                }
                if (g_rot_ref.size() != nodes.size())
                {
                    g_rot_ref.resize(nodes.size());
                    for (size_t n = 0; n < nodes.size(); n++)
                        g_rot_ref[n] = nodes[n].transform.rotation;
                }
                for (size_t n = 0; n < nodes.size(); n++)
                {
                    const ManusQuaternion& p = nodes[n].transform.rotation;
                    const ManusQuaternion& q = g_rot_ref[n];
                    float dot = p.x*q.x + p.y*q.y + p.z*q.z + p.w*q.w;
                    if (dot < 0) dot = -dot;
                    if (dot > 1.0f) dot = 1.0f;
                    float ang = 2.0f * std::acos(dot) * 57.2957795f;  // deg
                    if (ang > g_rot_max) g_rot_max = ang;
                }
                for (size_t n = 0; n < nodes.size(); n++)
                {
                    const ManusVec3& a = nodes[n].transform.position;
                    const ManusVec3& b = g_motion_ref[n];
                    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
                    float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (d > g_motion_max) g_motion_max = d;
                }
            }
        }
    }
}

static void OnSigInt(int) { g_stop = true; }

int main(int argc, char** argv)
{
    int run_seconds = 30;
    if (argc > 1) run_seconds = std::atoi(argv[1]);

    std::signal(SIGINT, OnSigInt);

    printf("[diag] CoreSdk_InitializeIntegrated...\n");
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode::SDKReturnCode_Success)
    {
        printf("[diag] FAILED to initialize\n");
        return 1;
    }

    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape);
    CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomics);
    CoreSdk_RegisterCallbackForRawSkeletonStream(OnSkeleton);

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side::Side_Right;
    vuh.up = AxisPolarity::AxisPolarity_PositiveZ;
    vuh.view = AxisView::AxisView_XFromViewer;
    vuh.unitScale = 1.0f;
    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode::SDKReturnCode_Success)
    {
        printf("[diag] FAILED coordinate system init\n");
        return 1;
    }

    printf("[diag] LookForHosts(integrated)...\n");
    if (CoreSdk_LookForHosts(1, false) != SDKReturnCode::SDKReturnCode_Success)
    {
        printf("[diag] FAILED LookForHosts\n");
        return 1;
    }
    uint32_t hosts = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&hosts);
    if (hosts == 0)
    {
        printf("[diag] no hosts found\n");
        return 1;
    }
    std::vector<ManusHost> host_list(hosts);
    CoreSdk_GetAvailableHostsFound(host_list.data(), hosts);
    if (CoreSdk_ConnectToHost(host_list[0]) == SDKReturnCode::SDKReturnCode_NotConnected)
    {
        printf("[diag] FAILED to connect\n");
        return 1;
    }
    printf("[diag] connected. observing streams for %d s...\n", run_seconds);

    // Must be called after connecting -- calling it before segfaults
    // (confirmed on SDK 3.1.1)
    const SDKReturnCode motion_rc = CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    printf("[diag] SetRawSkeletonHandMotion(Auto) rc=%d\n", (int)motion_rc);

    // User check: with no user in CoreLite the glove stays excluded and the
    // sensor stream never starts. If there is none, create one and assign the
    // glove manually.
    std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for landscape
    uint32_t user_count = 0;
    CoreSdk_GetNumberOfAvailableUsers(&user_count);
    printf("[diag] available users: %u (landscape says %u)\n", user_count, g_user_count.load());
    if (user_count == 0 && argc > 2 && std::string_view(argv[2]) == "--create-user")
    {
        printf("[diag] no user found. disabling auto-assignment and creating one...\n");
        CoreSdk_SetAutoUserAssignment(false);
        char user_name[] = "IsaacSimUser";
        uint32_t new_user_id = 0;
        SDKReturnCode rc = CoreSdk_AddUser(user_name, &new_user_id);
        printf("[diag] AddUser rc=%d id=%u\n", (int)rc, new_user_id);
        if (rc == SDKReturnCode::SDKReturnCode_Success)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (g_left_id)
            {
                rc = CoreSdk_AssignGloveToUser(new_user_id, g_left_id, Side::Side_Left);
                printf("[diag] AssignGloveToUser(L=0x%X) rc=%d\n", g_left_id.load(), (int)rc);
            }
            if (g_right_id)
            {
                rc = CoreSdk_AssignGloveToUser(new_user_id, g_right_id, Side::Side_Right);
                printf("[diag] AssignGloveToUser(R=0x%X) rc=%d\n", g_right_id.load(), (int)rc);
            }
        }
    }

    for (int t = 0; t < run_seconds && !g_stop; t++)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint32_t node_count, glove_id, skel_count;
        ManusVec3 pos;
        {
            std::lock_guard<std::mutex> lock(g_skel_mutex);
            node_count = g_last_node_count;
            glove_id = g_last_glove_id;
            skel_count = g_last_skel_count;
            pos = g_last_node1_pos;
        }
        printf("[t=%2d] landscape=%llu ergo=%llu skel=%llu | gloves=%u (L=0x%X R=0x%X) | "
               "last: skels=%u glove=0x%X nodes=%u node1=(%.3f, %.3f, %.3f) motion_max=%.1fmm rot_max=%.1fdeg ergo_span=%.2f\n",
               t + 1,
               (unsigned long long)g_landscape_events.load(),
               (unsigned long long)g_ergonomics_events.load(),
               (unsigned long long)g_skeleton_events.load(),
               g_glove_count.load(), g_left_id.load(), g_right_id.load(),
               skel_count, glove_id, node_count, pos.x, pos.y, pos.z);
        if ((t + 1) % 5 == 0)
        {
            std::lock_guard<std::mutex> lock(g_glove_detail_mutex);
            printf("%s", g_glove_detail);
        }
        {
            std::lock_guard<std::mutex> lock(g_ergo_mutex);
            printf("  [ergo R=0x%X] %.3f %.3f %.3f %.3f %.3f %.3f\n", g_ergo_glove, g_ergo_sample[0],
                   g_ergo_sample[1], g_ergo_sample[2], g_ergo_sample[3], g_ergo_sample[4], g_ergo_sample[5]);
        }
        fflush(stdout);
    }

    printf("[diag] shutting down...\n");
    CoreSdk_RegisterCallbackForRawSkeletonStream(nullptr);
    CoreSdk_RegisterCallbackForLandscapeStream(nullptr);
    CoreSdk_RegisterCallbackForErgonomicsStream(nullptr);
    CoreSdk_Disconnect();
    CoreSdk_ShutDown();
    printf("[diag] done\n");
    return 0;
}
