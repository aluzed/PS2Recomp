#pragma once

// Inline body of PS2Runtime::callStatic (see ps2_runtime.h). Separate header:
// it needs the EE scheduler's inline checkpoint, and ee_scheduler.h includes
// ps2_runtime.h.
#include "ps2_runtime.h"
#include "runtime/ee_scheduler.h"

extern const uint32_t g_ps2RecompiledFunctionTableBase;
extern const uint32_t g_ps2RecompiledFunctionTableSlotCount;

inline bool PS2Runtime::callStatic(uint8_t *rdram, R5900Context *ctx, uint32_t targetPc, uint32_t sourcePc,
                                   uint32_t fallthroughPc)
{
    ctx->pc = targetPc;
    // Every inter-function transfer is a deterministic EE safe point (as in dispatchGuestBranch).
    if (m_eeScheduler->checkpointDue(EeScheduler::kGuestDispatchCycles))
        return false;
    const uint32_t slot = (targetPc - g_ps2RecompiledFunctionTableBase) >> 2;
    RecompiledFunction fn = nullptr;
    if ((targetPc & 3u) == 0u && targetPc >= g_ps2RecompiledFunctionTableBase &&
        slot < g_ps2RecompiledFunctionTableSlotCount)
        fn = g_ps2RecompiledFunctionTable[slot];
    if (fn == nullptr) [[unlikely]]
        return callStaticMissing(rdram, ctx, targetPc, sourcePc, fallthroughPc);
    fn(rdram, ctx, this);
    if (m_stopRequested.load(std::memory_order_relaxed) || ctx->pc == 0u)
        return false;
    if (ctx->pc == targetPc)
        ctx->pc = fallthroughPc;
    return ctx->pc == fallthroughPc;
}
