#include "iop_vblank.h"

#include "../core/iop_cpu.h"
#include "../iop_emulator_const.h"
#include "../core/iop_kernel.h"
#include "../services/iop_rpc.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace ps2x::iop::detail
{
    IopVblank::IopVblank(IopKernel &kernel) noexcept
        : m_kernel(kernel)
    {
        reset();
    }

    void IopVblank::reset()
    {
        m_handlers.clear();
        m_nextStart = kVblankPeriodCycles;
        m_nextEnd = kVblankEndPhaseCycles;
        m_servicing = false;
    }

    void IopVblank::serviceDue(uint64_t currentCycle, IopGuestExecutor &executor)
    {
        if (m_servicing)
            return;
        while (m_nextStart <= currentCycle || m_nextEnd <= currentCycle)
        {
            const uint32_t edge = m_nextStart <= m_nextEnd ? 0u : 1u;
            if (edge == 0u)
                m_nextStart += kVblankPeriodCycles;
            else
                m_nextEnd += kVblankPeriodCycles;
            // Sound drivers run their sequencer/stream tick here (Street
            // Fighter EX3's IOPCTRL). As in vblank.irx (ps2sdk vblank.c):
            // lower priority value runs first, a handler returning 0 is unlinked.
            std::vector<Handler> due;
            for (const Handler &h : m_handlers)
                if (h.edge == edge && h.function != 0u)
                    due.push_back(h);
            std::stable_sort(due.begin(), due.end(), [](const Handler &a, const Handler &b)
                             { return a.priority < b.priority; });
            m_servicing = true;
            try
            {
                for (const Handler &h : due)
                {
                    const uint32_t keep = executor.executeGuestFunctionWithBudget(h.function, h.argument, 0u, 0u, 0u, h.gp, 200000u);
                    {
                        static uint32_t traced = 0;
                        if (std::getenv("PS2X_IMPORT_TRACE") && traced < 40u && ++traced)
                            std::fprintf(stderr, "[VBL] call fn=0x%x -> %u\n", h.function, keep);
                    }
                    if (keep == 0u)
                        m_handlers.erase(std::remove_if(m_handlers.begin(), m_handlers.end(), [&](const Handler &x)
                                                        { return x.edge == h.edge && x.function == h.function && x.argument == h.argument; }),
                                         m_handlers.end());
                }
            }
            catch (...)
            {
                m_servicing = false;
                throw;
            }
            m_servicing = false;
        }
    }

    uint64_t IopVblank::nextEventCycle(uint64_t fallback) const noexcept
    {
        if (m_handlers.empty())
            return fallback;
        return std::min({fallback, m_nextStart, m_nextEnd});
    }

    bool IopVblank::dispatchImport(uint16_t ordinal, IopCpuState &cpu, uint64_t currentCycle)
    {
        switch (ordinal)
        {
        case 4: // WaitVblankStart
        case 5: // WaitVblankEnd
        case 6: // WaitVblank
        case 7: // WaitNonVblank
        {
            const bool waitForEnd = ordinal == 5u || ordinal == 7u;
            const uint64_t phase = waitForEnd ? kVblankEndPhaseCycles : 0u;
            const uint64_t fieldStart = currentCycle - (currentCycle % kVblankPeriodCycles);
            uint64_t wakeCycle = fieldStart + phase;
            if (wakeCycle <= currentCycle)
                wakeCycle += kVblankPeriodCycles;
            m_kernel.delayCurrentUntil(wakeCycle, cpu);
            cpu.gpr[2] = 0u;
            return true;
        }
        case 8: // RegisterVblankHandler(edge, priority, handler, arg)
        {
            Handler h;
            h.edge = cpu.gpr[4] != 0u ? 1u : 0u;
            h.priority = static_cast<int32_t>(cpu.gpr[5]);
            h.function = cpu.gpr[6];
            h.argument = cpu.gpr[7];
            h.gp = cpu.gpr[28];
            m_handlers.push_back(h);
            if (std::getenv("PS2X_IMPORT_TRACE"))
                std::fprintf(stderr, "[VBL] register edge=%u prio=%d fn=0x%x arg=0x%x\n", h.edge, h.priority, h.function, h.argument);
            cpu.gpr[2] = 0u;
            return true;
        }
        case 9: // ReleaseVblankHandler(edge, handler)
        {
            const uint32_t edge = cpu.gpr[4] != 0u ? 1u : 0u;
            const uint32_t function = cpu.gpr[5];
            m_handlers.erase(std::remove_if(m_handlers.begin(), m_handlers.end(), [&](const Handler &h)
                                            { return h.edge == edge && h.function == function; }),
                             m_handlers.end());
            cpu.gpr[2] = 0u;
            return true;
        }
        default:
            return false;
        }
    }
}
