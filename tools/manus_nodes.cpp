// Dumps the node layout of the MANUS raw skeleton.
//
// Mapping the glove's 25 nodes into the MANO 21-point array that retargeting
// expects requires knowing which finger and which phalanx each node is. The
// SDK's NodeInfo carries chainType (finger), fingerJointType (phalanx) and
// parentId, so this reads them and prints a table.
//
// Build:
//   g++ -std=c++17 -I$SDK/include manus_nodes.cpp -L$SDK/lib -lManusSDK_Integrated \
//       -Wl,-rpath,$SDK/lib -lpthread -o manus_nodes

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

static std::atomic<uint32_t> g_glove_id{0};
static std::atomic<bool> g_have_skeleton{false};

static const char* ChainName(ChainType t)
{
    switch (t)
    {
        case ChainType_FingerThumb:  return "thumb";
        case ChainType_FingerIndex:  return "index";
        case ChainType_FingerMiddle: return "middle";
        case ChainType_FingerRing:   return "ring";
        case ChainType_FingerPinky:  return "little";
        case ChainType_Hand:         return "hand";
        case ChainType_Arm:          return "arm";
        default:                     return "?";
    }
}

static const char* JointName(FingerJointType t)
{
    switch (t)
    {
        case FingerJointType_Metacarpal:   return "metacarpal";
        case FingerJointType_Proximal:     return "proximal";
        case FingerJointType_Intermediate: return "intermediate";
        case FingerJointType_Distal:       return "distal";
        case FingerJointType_Tip:          return "tip";
        default:                           return "-";
    }
}

static void OnLandscape(const Landscape* landscape)
{
    const auto& gloves = landscape->gloveDevices;
    if (gloves.gloveCount > 0 && g_glove_id == 0)
        g_glove_id = gloves.gloves[0].id;
}

static void OnSkeleton(const SkeletonStreamInfo* info)
{
    if (info->skeletonsCount > 0)
        g_have_skeleton = true;
}

int main()
{
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success)
    { printf("FAILED InitializeIntegrated\n"); return 1; }

    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape);
    CoreSdk_RegisterCallbackForRawSkeletonStream(OnSkeleton);

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up = AxisPolarity_PositiveZ;
    vuh.view = AxisView_XFromViewer;
    vuh.unitScale = 1.0f;
    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success)
    { printf("FAILED coordinate system\n"); return 1; }

    if (CoreSdk_LookForHosts(1, false) != SDKReturnCode_Success)
    { printf("FAILED LookForHosts\n"); return 1; }
    uint32_t hosts = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&hosts);
    if (hosts == 0) { printf("no hosts\n"); return 1; }

    std::vector<ManusHost> host_list(hosts);
    CoreSdk_GetAvailableHostsFound(host_list.data(), hosts);
    if (CoreSdk_ConnectToHost(host_list[0]) == SDKReturnCode_NotConnected)
    { printf("FAILED ConnectToHost\n"); return 1; }

    // SDK 3.1.1: must be called after connecting (calling it before segfaults).
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);

    printf("waiting for glove and skeleton...\n");
    for (int i = 0; i < 200 && (!g_have_skeleton || g_glove_id == 0); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const uint32_t glove = g_glove_id;
    if (glove == 0 || !g_have_skeleton) { printf("no glove or skeleton\n"); return 1; }

    uint32_t count = 0;
    if (CoreSdk_GetRawSkeletonNodeCount(glove, count) != SDKReturnCode_Success)
    { printf("FAILED GetRawSkeletonNodeCount\n"); return 1; }

    std::vector<NodeInfo> info(count);
    if (CoreSdk_GetRawSkeletonNodeInfoArray(glove, info.data(), count) != SDKReturnCode_Success)
    { printf("FAILED GetRawSkeletonNodeInfoArray\n"); return 1; }

    printf("\nglove 0x%X -- %u nodes\n\n", glove, count);
    printf("%-6s %-8s %-8s %-8s %s\n", "idx", "nodeId", "parent", "side", "name (chain_joint)");
    for (uint32_t i = 0; i < count; i++)
    {
        const NodeInfo& n = info[i];
        const char* side = n.side == Side_Left ? "left" : (n.side == Side_Right ? "right" : "-");
        char name[64];
        if (n.chainType == ChainType_Hand || n.fingerJointType == FingerJointType_Invalid)
            snprintf(name, sizeof(name), "%s", ChainName(n.chainType));
        else
            snprintf(name, sizeof(name), "%s_%s", ChainName(n.chainType), JointName(n.fingerJointType));
        printf("%-6u %-8u %-8u %-8s %s\n", i, n.nodeId, n.parentId, side, name);
    }
    printf("\n");

    CoreSdk_RegisterCallbackForRawSkeletonStream(nullptr);
    CoreSdk_RegisterCallbackForLandscapeStream(nullptr);
    CoreSdk_ShutDown();
    return 0;
}
