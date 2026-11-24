#pragma once

#include <cstdint>
#include <vector>

namespace ps2x::iop::detail
{
    struct IopCpuState;
    class IopKernel;
    class IopGuestExecutor;

    class IopVblank
    {
    public:
        explicit IopVblank(IopKernel &kernel) noexcept;

        void reset();
        [[nodiscard]] bool dispatchImport(uint16_t ordinal, IopCpuState &cpu, uint64_t currentCycle);
        // Calls the registered VBlank start/end handlers whose edge is due.
        void serviceDue(uint64_t currentCycle, IopGuestExecutor &executor);
        [[nodiscard]] uint64_t nextEventCycle(uint64_t fallback) const noexcept;

    private:
        struct Handler
        {
            uint32_t edge = 0;     // 0 = VBlank start, 1 = VBlank end
            int32_t priority = 0;
            uint32_t function = 0;
            uint32_t argument = 0;
            uint32_t gp = 0;
        };

        IopKernel &m_kernel;
        std::vector<Handler> m_handlers;
        uint64_t m_nextStart = 0;
        uint64_t m_nextEnd = 0;
        bool m_servicing = false;
    };
}
