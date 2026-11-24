#pragma once

#include <atomic>
#include <cstdint>

namespace ps2x::iop
{
    // Host-side control of the emulated SPU2 output. Muted voices keep running
    // (decoding, NAX/ENDX, IRQs), so the game's sound driver is unaffected;
    // they only stop contributing to the mix. Used to replace a game's music
    // with a host-played track.
    inline std::atomic<uint32_t> g_spu2MutedVoices[2]{};

    inline void setSpu2MutedVoices(int core, uint32_t mask) noexcept
    {
        g_spu2MutedVoices[core & 1].store(mask & 0xFFFFFFu, std::memory_order_relaxed);
    }

    [[nodiscard]] inline uint32_t spu2MutedVoices(int core) noexcept
    {
        return g_spu2MutedVoices[core & 1].load(std::memory_order_relaxed);
    }

    // Last music track the game asked its IOP music driver to play (Street
    // Fighter EX3: SIF RPC 0x502 function 0x102, track number in word 2), and a
    // counter bumped on every such request.
    inline std::atomic<uint32_t> g_musicTrack{0xFFFFFFFFu};
    inline std::atomic<uint32_t> g_musicRequests{0};

    inline void noteMusicRequest(uint32_t track) noexcept
    {
        g_musicTrack.store(track, std::memory_order_relaxed);
        g_musicRequests.fetch_add(1u, std::memory_order_relaxed);
    }
}
