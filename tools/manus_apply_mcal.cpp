// CLI that applies MANUS .mcal calibration profiles to the left and right gloves.
//
// SDKClient's [L] (Load) reads one fixed filename and applies it only to the
// currently selected hand, so this exists to apply both hands in one go. It
// connects to the dongle directly in Core Integrated mode, takes the left and
// right glove IDs from the landscape, and calls CoreSdk_SetGloveCalibration.
// CoreLite saves the applied calibration to its settings file automatically.
//
// Build:
//   g++ -std=c++17 -I$SDK/include manus_apply_mcal.cpp -L$SDK/lib -lManusSDK_Integrated \
//       -Wl,-rpath,$SDK/lib -lpthread -o manus_apply_mcal
// Usage:
//   ./manus_apply_mcal <left.mcal> <right.mcal>

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>
#include <vector>

static std::atomic<uint32_t> g_left_id{0};
static std::atomic<uint32_t> g_right_id{0};

static void OnLandscape(const Landscape* landscape)
{
    const auto& gloves = landscape->gloveDevices;
    for (uint32_t i = 0; i < gloves.gloveCount; i++)
    {
        if (gloves.gloves[i].side == Side::Side_Left) g_left_id = gloves.gloves[i].id;
        if (gloves.gloves[i].side == Side::Side_Right) g_right_id = gloves.gloves[i].id;
    }
}

static std::vector<unsigned char> read_file(const char* path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        return {};
    auto size = f.tellg();
    f.seekg(0);
    std::vector<unsigned char> data(size);
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

static const char* rc_name(SetGloveCalibrationReturnCode rc)
{
    switch (rc)
    {
    case SetGloveCalibrationReturnCode_Success: return "Success";
    case SetGloveCalibrationReturnCode_Error: return "Error";
    case SetGloveCalibrationReturnCode_VersionError: return "VersionError";
    case SetGloveCalibrationReturnCode_WrongSideError: return "WrongSideError";
    case SetGloveCalibrationReturnCode_GloveNotFoundError: return "GloveNotFoundError";
    case SetGloveCalibrationReturnCode_UserServiceError: return "UserServiceError";
    case SetGloveCalibrationReturnCode_DeserializationError: return "DeserializationError";
    default: return "Unknown";
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        printf("usage: %s <left.mcal> <right.mcal>\n", argv[0]);
        return 1;
    }
    auto left_data = read_file(argv[1]);
    auto right_data = read_file(argv[2]);
    if (left_data.empty() || right_data.empty())
    {
        printf("[apply] cannot read files: %s / %s\n", argv[1], argv[2]);
        return 1;
    }
    printf("[apply] left=%s (%zu bytes), right=%s (%zu bytes)\n", argv[1], left_data.size(), argv[2],
           right_data.size());

    if (CoreSdk_InitializeIntegrated() != SDKReturnCode::SDKReturnCode_Success)
    {
        printf("[apply] SDK initialization failed\n");
        return 1;
    }
    CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape);

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side::Side_Right;
    vuh.up = AxisPolarity::AxisPolarity_PositiveZ;
    vuh.view = AxisView::AxisView_XFromViewer;
    vuh.unitScale = 1.0f;
    CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true);

    if (CoreSdk_LookForHosts(1, false) != SDKReturnCode::SDKReturnCode_Success)
    {
        printf("[apply] LookForHosts failed\n");
        return 1;
    }
    uint32_t hosts = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&hosts);
    if (hosts == 0)
    {
        printf("[apply] no host found\n");
        return 1;
    }
    std::vector<ManusHost> host_list(hosts);
    CoreSdk_GetAvailableHostsFound(host_list.data(), hosts);
    if (CoreSdk_ConnectToHost(host_list[0]) == SDKReturnCode::SDKReturnCode_NotConnected)
    {
        printf("[apply] connection failed\n");
        return 1;
    }

    printf("[apply] waiting for gloves (up to 20 s)...\n");
    for (int i = 0; i < 40 && (g_left_id == 0 || g_right_id == 0); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("[apply] L=0x%X R=0x%X\n", g_left_id.load(), g_right_id.load());

    int fail = 0;
    if (g_left_id != 0)
    {
        SetGloveCalibrationReturnCode rc;
        SDKReturnCode api = CoreSdk_SetGloveCalibration(g_left_id, left_data.data(), (uint32_t)left_data.size(), &rc);
        printf("[apply] LEFT  0x%X: api=%d result=%s\n", g_left_id.load(), (int)api, rc_name(rc));
        fail += (rc != SetGloveCalibrationReturnCode_Success);
    }
    else
    {
        printf("[apply] LEFT glove not found -- skipping\n");
        fail++;
    }
    if (g_right_id != 0)
    {
        SetGloveCalibrationReturnCode rc;
        SDKReturnCode api =
            CoreSdk_SetGloveCalibration(g_right_id, right_data.data(), (uint32_t)right_data.size(), &rc);
        printf("[apply] RIGHT 0x%X: api=%d result=%s\n", g_right_id.load(), (int)api, rc_name(rc));
        fail += (rc != SetGloveCalibrationReturnCode_Success);
    }
    else
    {
        printf("[apply] RIGHT glove not found -- skipping\n");
        fail++;
    }

    // Give CoreLite time to save its settings
    std::this_thread::sleep_for(std::chrono::seconds(3));
    CoreSdk_RegisterCallbackForLandscapeStream(nullptr);
    CoreSdk_Disconnect();
    CoreSdk_ShutDown();
    printf("[apply] %s\n", fail == 0 ? "done (both hands succeeded)" : "partial failure");
    return fail;
}
