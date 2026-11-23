#include "runtime/vu/vu1_kzvu_bridge.h"

#if !defined(PS2X_ENABLE_KZVU)

bool installKzvuVu1(PS2Runtime &) { return false; }
bool runKzvuVu0(R5900Context *, uint32_t) { return false; }

#else

#include "ps2_runtime.h"
#include "runtime/ee_scheduler.h"

#include "kzvu.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// Glue modelled on PS2Recomp-Fixer's dod_kzvu.cpp (GPL-3.0): VU1 memory becomes
// kzvu's, MSCAL/MSCNT run the program to its E bit, XGKICK packets go to the GIF
// as PATH1. Everything runs on the game thread, synchronously, as before.
namespace
{
    PS2Runtime *g_runtime = nullptr;
    bool g_vu0Active = false;
    uint64_t g_vu0CodeGeneration = ~0ull;
    uint64_t g_codeGeneration = ~0ull;
    constexpr uint32_t kBudget = 1u << 24; // programs run to their E bit

    int clampModeFromEnvironment(const char *name)
    {
        const char *value = std::getenv(name);
        if (!value || !*value)
            return 1;
        if (value[0] < '0' || value[0] > '3' || value[1] != '\0')
        {
            std::fprintf(stderr, "[vu1] %s='%s' is not 0..3; using clamp mode 1\n", name, value);
            return 1;
        }
        return value[0] - '0';
    }

    void onXgkick(void *, const uint8_t *packet, uint32_t bytes, uint32_t)
    {
        g_runtime->memory().submitGifPacket(GifPathId::Path1, packet, bytes);
    }

    R5900Context *eeContext()
    {
        return g_runtime->eeScheduler().currentContext();
    }

    void prepare(uint32_t top, uint32_t itop)
    {
        // VIF1 MPG writes VU1 micro memory directly and bumps the generation.
        const uint64_t generation = g_runtime->memory().getVU1CodeGeneration();
        if (generation != g_codeGeneration)
        {
            kzvuMicroWritten(0, kKzvuCodeSize);
            g_codeGeneration = generation;
        }
        const R5900Context *ctx = eeContext();
        kzvuSetFBRST(ctx ? ctx->vu0_fbrst : 0u);
        kzvuSetTop(top, itop);
    }

    void finish()
    {
        for (int i = 0; i < 16 && kzvuRunning() && (kzvuVpuStat() & 0x0600u) == 0u; ++i)
            kzvuContinue(kBudget);
        if (R5900Context *ctx = eeContext())
            ctx->vu0_vpu_stat = (ctx->vu0_vpu_stat & ~0x0600u) | (kzvuVpuStat() & 0x0600u);
    }
}

bool installKzvuVu1(PS2Runtime &runtime)
{
    const char *env = std::getenv("PS2X_VU1_CORE");
    const std::string mode = env && *env ? env : "microvu";
    if (mode == "builtin")
    {
        std::fprintf(stderr, "[vu1] runtime VU1Interpreter (PS2X_VU1_CORE=builtin)\n");
        return false;
    }
    KzvuConfig config;
    config.useJit = mode != "interp";
    config.clampMode = clampModeFromEnvironment("PS2X_VU1_CLAMP");
    config.vu0ClampMode = 1;
    config.iBitHack = false;
    config.flagHack = true;
    config.xgkick = &onXgkick;
    std::string error;
    if (!kzvuInit(config, &error))
    {
        std::fprintf(stderr, "[vu1] kzvu failed to start (%s); keeping VU1Interpreter\n", error.c_str());
        return false;
    }
    g_runtime = &runtime;
    PS2Memory &memory = runtime.memory();
    memory.adoptVU1Buffers(kzvuCodeMem(), kzvuDataMem());
    memory.setVu1MscalCallback([](uint32_t startPC, uint32_t top, uint32_t itop)
                               {
                                   prepare(top, itop);
                                   kzvuExecute(startPC, kBudget);
                                   finish(); });
    memory.setVu1MscntCallback([](uint32_t top, uint32_t itop)
                               {
                                   prepare(top, itop);
                                   kzvuExecute(0xFFFFFFFFu, kBudget);
                                   finish(); });
    const char *vu0Env = std::getenv("PS2X_VU0_CORE");
    if (!(vu0Env && std::string(vu0Env) == "builtin"))
    {
        memory.adoptVU0Buffers(kzvu0CodeMem(), kzvu0DataMem());
        g_vu0Active = true;
    }
    std::fprintf(stderr, "[vu0] %s\n", g_vu0Active ? "microVU0 (kzvu)" : "runtime VU1Interpreter (PS2X_VU0_CORE=builtin)");
    std::fprintf(stderr, "[vu1] %s (kzvu), clamp mode %d\n",
                 config.useJit ? "microVU recompiler" : "PCSX2 VU interpreter", config.clampMode);
    return true;
}

bool runKzvuVu0(R5900Context *ctx, uint32_t startPc)
{
    if (!g_vu0Active || !ctx)
        return false;
    const uint64_t generation = g_runtime->memory().getVU0CodeGeneration();
    if (generation != g_vu0CodeGeneration)
    {
        kzvu0MicroWritten(0, kKzvu0CodeSize);
        g_vu0CodeGeneration = generation;
    }
    // The EE's COP2 file is plain memory in R5900Context: kzvu reads and
    // writes it in place.
    Kzvu0Host host{};
    host.vf = reinterpret_cast<uint32_t(*)[4]>(ctx->vu0_vf);
    host.vi = ctx->vi;
    host.acc = reinterpret_cast<uint32_t *>(&ctx->vu0_acc);
    host.status = &ctx->vu0_status;
    host.mac = &ctx->vu0_mac_flags;
    host.clip = &ctx->vu0_clip_flags;
    host.clip2 = &ctx->vu0_clip_flags2;
    host.r = reinterpret_cast<uint32_t *>(&ctx->vu0_r);
    host.i = reinterpret_cast<uint32_t *>(&ctx->vu0_i);
    host.q = reinterpret_cast<uint32_t *>(&ctx->vu0_q);
    const Kzvu0CallOut out = kzvu0Call(host, startPc, kBudget, ctx->vu0_fbrst);
    ctx->vu0_tpc = out.tpc;
    ctx->vu0_pc = out.tpc;
    // Programs run to completion here: never report VBS0 busy.
    ctx->vu0_vpu_stat = (ctx->vu0_vpu_stat & ~0x7u) | (out.vpuStat & 0x6u);
    return true;
}

#endif // PS2X_ENABLE_KZVU
