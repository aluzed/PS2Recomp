// Based on Blackline Interactive implementation
#include <algorithm>
#include <atomic>
#include <iostream>
#include <chrono>
#include <utility>
#include <array>
#include <cstdlib>
#include "runtime/ps2_memory.h"
#include <cstring>

enum VIFCmd : uint8_t
{
    VIF_NOP = 0x00,
    VIF_STCYCL = 0x01,
    VIF_OFFSET = 0x02,
    VIF_BASE = 0x03,
    VIF_ITOP = 0x04,
    VIF_STMOD = 0x05,
    VIF_MSKPATH3 = 0x06,
    VIF_MARK = 0x07,
    VIF_FLUSHE = 0x10,
    VIF_FLUSH = 0x11,
    VIF_FLUSHA = 0x13,
    VIF_MSCAL = 0x14,
    VIF_MSCALF = 0x15,
    VIF_MSCNT = 0x17,
    VIF_STMASK = 0x20,
    VIF_STROW = 0x30,
    VIF_STCOL = 0x31,
    VIF_MPG = 0x4A,
    VIF_DIRECT = 0x50,
    VIF_DIRECTHL = 0x51,
};

namespace
{
    constexpr uint8_t kGifFmtImage = 2u;

    // Bytes of source data per vector, by VN<<2|VL (PCSX2 nVifT).
    constexpr uint8_t kVifUnpackSourceBytes[16] = {4, 2, 1, 0, 8, 4, 2, 0, 12, 6, 3, 0, 16, 8, 4, 2};

    bool legacyVif1UnpackRequested()
    {
        static const bool legacy = []
        {
            const char *v = std::getenv("PS2X_VIF_UNPACK");
            return v && std::strcmp(v, "legacy") == 0;
        }();
        return legacy;
    }

    // VIF1 UNPACK with PCSX2's semantics (pcsx2/Vif_Unpack.cpp). Ported from
    // PS2Recomp-Fixer / KillzoneRecomp patch 0010 (GPL-3.0), including Fixer's
    // per-format specialisation (dod_vif1_unpack.h): VN/VL, the mask bit and
    // STMOD are template parameters, so the format, mask and mode switches
    // compile away.
    //  - WL = 0 means 256; fill mode is CL < WL: the source only advances for
    //    the first CL writes of a block, the other writes reuse the next vector
    //    (normally masked to ROW/COL); skipping mode adds (CL - WL) qwords.
    //  - S-xx broadcasts; V2 writes x,y,x,y; V3 reads four elements (W is the
    //    next vector's first element); V4-5 expands 5:5:5:1 to 8-bit channels
    //    and ignores STMOD.
    //  - STMOD 1 adds ROW, 2 adds and stores ROW, 3 stores the data into ROW.
    //  - Mask row and COL are selected by the write cycle (min(cl, 3)).
    //  - The destination wraps in VU memory.
    //  - CYCLE = 0 (reset state, no STCYCL yet) is a plain linear transfer.
    // Reads past dataBytes return zero.
    template <uint32_t VNVL, bool MASK, uint32_t MODE>
    void vif1UnpackFormat(uint8_t *vuMem, uint32_t vuMemSize, VIFRegisters &regs, uint32_t cmd,
                          const uint8_t *data, uint32_t dataBytes)
    {
        constexpr uint32_t vsize = kVifUnpackSourceBytes[VNVL];
        if constexpr (vsize == 0u)
        {
            (void)vuMem, (void)vuMemSize, (void)regs, (void)cmd, (void)data, (void)dataBytes;
            return; // invalid unpack (PCSX2 only warns)
        }
        else
        {
            constexpr uint32_t vl = VNVL & 3u;
            constexpr uint32_t vn = VNVL >> 2;
            constexpr uint32_t elementBytes = 4u >> vl;
            const bool usn = (cmd & 0x4000u) != 0u;
            uint32_t num = (cmd >> 16) & 0xFFu;
            if (num == 0u)
                num = 256u;
            const bool linear = (regs.cycle & 0xFFFFu) == 0u;
            const uint32_t cl = linear ? 1u : (regs.cycle & 0xFFu);
            const uint32_t wl = linear ? 1u : (((regs.cycle >> 8) & 0xFFu) ? ((regs.cycle >> 8) & 0xFFu) : 256u);
            const bool isFill = cl < wl;
            const uint32_t memMask = vuMemSize - 16u;
            uint32_t addr = cmd & 0x3FFu;
            if ((cmd & 0x8000u) != 0u)
                addr += regs.tops;
            addr = (addr << 4) & memMask;
            const uint32_t skipBytes = isFill ? 0u : (cl - wl) * 16u;

            auto element = [&](uint32_t offset, uint32_t index) -> uint32_t
            {
                const uint32_t at = offset + index * elementBytes;
                if (at + elementBytes > dataBytes)
                    return 0u;
                if constexpr (vl == 0u)
                {
                    uint32_t v;
                    std::memcpy(&v, data + at, 4);
                    return v;
                }
                else if constexpr (vl == 1u)
                {
                    uint16_t v;
                    std::memcpy(&v, data + at, 2);
                    return usn ? static_cast<uint32_t>(v) : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(v)));
                }
                else
                {
                    const uint8_t v = data[at];
                    return usn ? static_cast<uint32_t>(v) : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)));
                }
            };

            auto write = [&](uint32_t *dest, uint32_t f, uint32_t value)
            {
                if constexpr (MODE == 1u)
                    dest[f] = value + regs.row[f];
                else if constexpr (MODE == 2u)
                {
                    regs.row[f] += value;
                    dest[f] = regs.row[f];
                }
                else if constexpr (MODE == 3u)
                {
                    regs.row[f] = value;
                    dest[f] = value;
                }
                else
                    dest[f] = value;
            };

            uint32_t src = 0u;
            uint32_t vcl = 0u;
            for (uint32_t n = 0; n < num; ++n)
            {
                uint32_t in[4];
                if constexpr (VNVL == 15u)
                {
                    uint16_t v = 0;
                    if (src + 2u <= dataBytes)
                        std::memcpy(&v, data + src, 2);
                    in[0] = (v & 0x001Fu) << 3;
                    in[1] = (v & 0x03E0u) >> 2;
                    in[2] = (v & 0x7C00u) >> 7;
                    in[3] = (v & 0x8000u) >> 8;
                }
                else if constexpr (vn == 0u)
                {
                    in[0] = in[1] = in[2] = in[3] = element(src, 0);
                }
                else if constexpr (vn == 1u)
                {
                    in[0] = in[2] = element(src, 0);
                    in[1] = in[3] = element(src, 1);
                }
                else if constexpr (vl == 0u)
                {
                    // V3-32/V4-32: one copy when the four words are in range.
                    if (src + 16u <= dataBytes)
                        std::memcpy(in, data + src, 16);
                    else
                        for (uint32_t c = 0; c < 4u; ++c)
                            in[c] = element(src, c);
                }
                else
                {
                    for (uint32_t c = 0; c < 4u; ++c)
                        in[c] = element(src, c);
                }

                uint32_t *dest = reinterpret_cast<uint32_t *>(vuMem + (addr & memMask));
                if constexpr (!MASK)
                {
                    for (uint32_t f = 0; f < 4u; ++f)
                        write(dest, f, in[f]);
                }
                else
                {
                    const uint32_t maskCycle = vcl < 3u ? vcl : 3u;
                    for (uint32_t f = 0; f < 4u; ++f)
                    {
                        switch ((regs.mask >> (maskCycle * 8u + f * 2u)) & 3u)
                        {
                        case 0: write(dest, f, in[f]); break;
                        case 1: dest[f] = regs.row[f]; break;
                        case 2: dest[f] = regs.col[maskCycle]; break;
                        default: break; // write protect
                        }
                    }
                }

                addr += 16u;
                ++vcl;
                if (isFill)
                {
                    if (vcl <= cl)
                        src += vsize;
                    else if (vcl == wl)
                        vcl = 0u;
                }
                else
                {
                    src += vsize;
                    if (vcl >= wl)
                    {
                        addr += skipBytes;
                        vcl = 0u;
                    }
                }
            }
            regs.num = 0u;
        }
    }

    using VifUnpackFn = void (*)(uint8_t *, uint32_t, VIFRegisters &, uint32_t, const uint8_t *, uint32_t);

    // Index: VN<<5 | VL<<3 | mask bit<<2 | STMOD.
    template <size_t... K>
    constexpr std::array<VifUnpackFn, sizeof...(K)> makeVifUnpackTable(std::index_sequence<K...>)
    {
        return {{&vif1UnpackFormat<static_cast<uint32_t>(K >> 3), ((K >> 2) & 1u) != 0u, static_cast<uint32_t>(K & 3u)>...}};
    }

    void vif1UnpackPcsx2(uint8_t *vuMem, uint32_t vuMemSize, VIFRegisters &regs, uint32_t cmd,
                         const uint8_t *data, uint32_t dataBytes)
    {
        static constexpr auto table = makeVifUnpackTable(std::make_index_sequence<128>{});
        const uint32_t upk = (cmd >> 24) & 0x1Fu;
        const uint32_t vnvl = upk & 0xFu;
        const uint32_t mode = (vnvl == 15u) ? 0u : (regs.mode & 3u); // V4-5 ignores STMOD
        table[(vnvl << 3) | (((upk >> 4) & 1u) << 2) | mode](vuMem, vuMemSize, regs, cmd, data, dataBytes);
    }

    uint32_t pendingGifImageQwc(const uint8_t *data, uint32_t sizeBytes)
    {
        if (!data || sizeBytes < 16u)
            return 0u;

        uint32_t offset = 0u;
        while (offset + 16u <= sizeBytes)
        {
            uint64_t tagLo = 0u;
            std::memcpy(&tagLo, data + offset, sizeof(tagLo));
            offset += 16u;

            const uint32_t nloop = static_cast<uint32_t>(tagLo & 0x7FFFu);
            const uint8_t flg = static_cast<uint8_t>((tagLo >> 58) & 0x3u);
            uint32_t nreg = static_cast<uint32_t>((tagLo >> 60) & 0xFu);
            if (nreg == 0u)
                nreg = 16u;

            uint64_t payloadBytes = 0u;
            if (flg == 0u) // PACKED
            {
                payloadBytes = static_cast<uint64_t>(nloop) * nreg * 16ull;
            }
            else if (flg == 1u) // REGLIST, padded to a quadword
            {
                payloadBytes = static_cast<uint64_t>(nloop) * nreg * 8ull;
                payloadBytes = (payloadBytes + 15ull) & ~15ull;
            }
            else if (flg == kGifFmtImage || flg == 3u) // FLG=3 ("disabled" in the GS manual) behaves exactly like IMAGE: NLOOP qwords of image data.
            {
                payloadBytes = static_cast<uint64_t>(nloop) * 16ull;
                const uint64_t availableBytes = sizeBytes - offset;
                if (payloadBytes > availableBytes)
                {
                    return static_cast<uint32_t>((payloadBytes - availableBytes) / 16ull);
                }
            }
            else
            {
                return 0u;
            }

            if (payloadBytes > static_cast<uint64_t>(sizeBytes - offset))
                return 0u;
            offset += static_cast<uint32_t>(payloadBytes);
        }

        return 0u;
    }
}

void PS2Memory::processVIF0Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (sizeBytes == 0u || srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF0Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF0Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (sizeBytes == 0u)
        return;

    uint32_t pos = 0;
    while (pos + 4 <= sizeBytes)
    {
        uint32_t cmd = 0u;
        std::memcpy(&cmd, data + pos, sizeof(cmd));
        pos += 4u;

        const uint8_t opcode = static_cast<uint8_t>((cmd >> 24) & 0x7Fu);
        const uint16_t imm = static_cast<uint16_t>(cmd & 0xFFFFu);
        const uint8_t num = static_cast<uint8_t>((cmd >> 16) & 0xFFu);
        const bool irq = (cmd & 0x80000000u) != 0u;

        vif0_regs.code = cmd;
        vif0_regs.num = num;
        if (irq)
            vif0_regs.stat |= (1u << 11);

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif0_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            vif0_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif0_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif0_regs.mark = imm;
            vif0_regs.stat |= (1u << 6);
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4u > sizeBytes)
                break;
            std::memcpy(&vif0_regs.mask, data + pos, sizeof(vif0_regs.mask));
            pos += 4u;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.row, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16u > sizeBytes)
                break;
            std::memcpy(vif0_regs.col, data + pos, 16u);
            pos += 16u;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            const uint32_t destAddr = static_cast<uint32_t>(imm & 0x1FFu) * 8u;
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            uint32_t copyBytes = 0u;
            if (m_vu0Code && destAddr < PS2_VU0_CODE_SIZE && mpgBytes > 0u)
            {
                copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU0_CODE_SIZE)
                    copyBytes = PS2_VU0_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                {
                    std::memcpy(m_vu0Code + destAddr, data + pos, copyBytes);
                    markVU0CodeModified();
                }
            }

            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if ((opcode & 0x60u) == 0x60u)
        {
            const uint8_t vn = static_cast<uint8_t>((opcode >> 2) & 0x3u);
            const uint8_t vl = static_cast<uint8_t>(opcode & 0x3u);
            const int components = static_cast<int>(vn) + 1;
            int bitsPerComponent = 32;
            switch (vl)
            {
            case 0:
                bitsPerComponent = 32;
                break;
            case 1:
                bitsPerComponent = 16;
                break;
            case 2:
                bitsPerComponent = 8;
                break;
            case 3:
                bitsPerComponent = (vn == 3u) ? 4 : 16;
                break;
            default:
                break;
            }
            const int bitsPerVector = (vl == 3u && vn == 3u) ? 16 : (components * bitsPerComponent);
            uint32_t bytesPerVector = static_cast<uint32_t>((bitsPerVector + 7) / 8);
            const uint32_t writeVectorCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            uint32_t cl = vif0_regs.cycle & 0xFFu;
            uint32_t wl = (vif0_regs.cycle >> 8) & 0xFFu;
            if (cl == 0u)
                cl = 1u;
            if (wl == 0u)
                wl = 1u;
            uint32_t sourceVectorCount = writeVectorCount;
            if (cl < wl)
            {
                const uint32_t fullBlocks = writeVectorCount / wl;
                uint32_t remainder = writeVectorCount % wl;
                if (remainder > cl)
                    remainder = cl;
                sourceVectorCount = fullBlocks * cl + remainder;
            }
            uint32_t totalBytes = sourceVectorCount * bytesPerVector;
            totalBytes = (totalBytes + 3u) & ~3u;

            if (m_vu0Data && pos + totalBytes <= sizeBytes && vl == 0u)
            {
                uint32_t vuAddr = static_cast<uint32_t>(imm & 0x3FFu);
                if ((imm & 0x8000u) != 0u)
                    vuAddr = (vuAddr + (vif0_regs.tops & 0x3FFu)) & 0x3FFu;
                const uint8_t *srcBase = data + pos;
                uint32_t srcIndex = 0u;
                for (uint32_t writeIndex = 0; writeIndex < writeVectorCount; ++writeIndex)
                {
                    const uint32_t cyclePos = writeIndex % wl;
                    const bool sourceAvailable = (cl >= wl) || (cyclePos < cl);
                    uint32_t destVec = (cl >= wl) ? ((vuAddr + (writeIndex / wl) * cl + cyclePos) & 0x3FFu)
                                                  : ((vuAddr + writeIndex) & 0x3FFu);
                    const uint32_t destOff = destVec * 16u;
                    if (destOff + 16u > PS2_VU0_DATA_SIZE)
                    {
                        if (sourceAvailable && srcIndex < sourceVectorCount)
                            ++srcIndex;
                        continue;
                    }
                    if (!sourceAvailable || srcIndex >= sourceVectorCount)
                        continue;
                    const uint8_t *srcVec = srcBase + srcIndex * bytesPerVector;
                    ++srcIndex;
                    uint32_t lanes[4] = {0u, 0u, 0u, 0u};
                    std::memcpy(lanes, m_vu0Data + destOff, sizeof(lanes));
                    const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                    for (uint32_t c = 0; c < limit; ++c)
                    {
                        uint32_t scalar = 0u;
                        std::memcpy(&scalar, srcVec + c * 4u, sizeof(scalar));
                        lanes[c] = scalar;
                    }
                    _mm_storeu_si128(reinterpret_cast<__m128i *>(m_vu0Data + destOff), _mm_loadu_si128(reinterpret_cast<const __m128i *>(lanes)));
                }
            }
            pos += totalBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else
        {
            break;
        }
    }
}

void PS2Memory::processVIF1Data(uint32_t srcPhys, uint32_t sizeBytes)
{
    if (sizeBytes == 0u || srcPhys >= PS2_RAM_SIZE)
        return;

    const uint64_t requestedEnd = static_cast<uint64_t>(srcPhys) + static_cast<uint64_t>(sizeBytes);
    if (requestedEnd > static_cast<uint64_t>(PS2_RAM_SIZE))
        sizeBytes = PS2_RAM_SIZE - srcPhys;

    processVIF1Data(m_rdram + srcPhys, sizeBytes);
}

void PS2Memory::processVIF1DataNow(const uint8_t *data, uint32_t sizeBytes)
{
    if (sizeBytes == 0u)
        return;

    uint32_t pos = 0;

    while (pos + 4 <= sizeBytes)
    {
        if (m_vif1PendingPath2ImageQwc != 0u)
        {
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            if (availableQw == 0u)
            {
                break;
            }

            const uint32_t chunkQw = std::min<uint32_t>(m_vif1PendingPath2ImageQwc, availableQw);
            std::vector<uint8_t> imagePacket(16u + static_cast<size_t>(chunkQw) * 16u, 0u);
            const uint64_t imageTag =
                static_cast<uint64_t>(chunkQw & 0x7FFFu) |
                ((m_vif1PendingPath2ImageQwc == chunkQw) ? (1ull << 15) : 0ull) |
                (static_cast<uint64_t>(kGifFmtImage) << 58);
            std::memcpy(imagePacket.data(), &imageTag, sizeof(imageTag));
            std::memcpy(imagePacket.data() + 16u, data + pos, static_cast<size_t>(chunkQw) * 16u);
            submitGifPacket(GifPathId::Path2, imagePacket.data(), static_cast<uint32_t>(imagePacket.size()), true, m_vif1PendingPath2DirectHl);

            pos += chunkQw * 16u;
            m_vif1PendingPath2ImageQwc -= chunkQw;
            if (m_vif1PendingPath2ImageQwc == 0u)
            {
                m_vif1PendingPath2DirectHl = false;
            }
            continue;
        }

        uint32_t cmd;
        memcpy(&cmd, data + pos, 4);
        pos += 4;

        uint8_t opcode = (cmd >> 24) & 0x7F;
        uint16_t imm = cmd & 0xFFFF;
        uint8_t num = (cmd >> 16) & 0xFF;
        const bool irq = (cmd & 0x80000000u) != 0u;

        // Track most-recent command for VIFn_CODE emulation.
        vif1_regs.code = cmd;
        vif1_regs.num = num;
        if (irq)
            vif1_regs.stat |= (1u << 11); // INT

        if (opcode == VIF_NOP)
        {
            continue;
        }
        else if (opcode == VIF_STCYCL)
        {
            vif1_regs.cycle = imm;
            continue;
        }
        else if (opcode == VIF_OFFSET)
        {
            // VIF double-buffer setup. OFFSET clears DBF and resets TOPS to BASE.
            // Do not rewrite BASE from the previous TOPS value.
            vif1_regs.ofst = imm & 0x3FFu;
            vif1_regs.tops = vif1_regs.base & 0x3FFu;
            vif1_regs.stat &= ~(1u << 7); // clear DBF
            continue;
        }
        else if (opcode == VIF_BASE)
        {
            // BASE only updates the base register. TOPS changes on OFFSET/MSCAL.
            vif1_regs.base = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_ITOP)
        {
            // ITOP VIFcode writes pending ITOPS; VU XITOP observes it after MSCAL/MSCNT.
            vif1_regs.itops = imm & 0x3FFu;
            continue;
        }
        else if (opcode == VIF_STMOD)
        {
            vif1_regs.mode = imm & 3u;
            continue;
        }
        else if (opcode == VIF_MSKPATH3)
        {
            // VIF command docs: MSKPATH3 uses IMMEDIATE bit 15.
            std::lock_guard<std::recursive_mutex> gifLock(m_gifMutex);
            const bool wasMasked = m_path3Masked;
            m_path3Masked = (imm & 0x8000u) != 0u;
            if (wasMasked && !m_path3Masked)
                flushMaskedPath3Packets();
            continue;
        }
        else if (opcode == VIF_MARK)
        {
            vif1_regs.mark = imm;
            vif1_regs.stat |= (1u << 6); // MRK
            continue;
        }
        else if (opcode == VIF_FLUSHE || opcode == VIF_FLUSH || opcode == VIF_FLUSHA)
        {
            continue;
        }
        else if (opcode == VIF_MSCAL || opcode == VIF_MSCALF)
        {
            uint32_t startPC = (uint32_t)imm * 8u;

            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            if (m_vu1MscalCallback)
                m_vu1MscalCallback(startPC, runTop, runItop);
            continue;
        }
        else if (opcode == VIF_MSCNT)
        {
            const uint32_t runTop = vif1_regs.tops & 0x3FFu;
            const uint32_t runItop = vif1_regs.itops & 0x3FFu;
            vif1_regs.top = runTop;
            vif1_regs.itop = runItop;

            const bool dbf = (vif1_regs.stat & (1u << 7)) != 0u;
            if (dbf)
                vif1_regs.tops = vif1_regs.base & 0x3FFu;
            else
                vif1_regs.tops = (vif1_regs.base + vif1_regs.ofst) & 0x3FFu;
            vif1_regs.stat ^= (1u << 7); // toggle DBF

            if (m_vu1MscntCallback)
                m_vu1MscntCallback(runTop, runItop);
            continue;
        }
        else if (opcode == VIF_STMASK)
        {
            if (pos + 4 > sizeBytes)
                break;
            uint32_t maskValue = 0;
            std::memcpy(&maskValue, data + pos, sizeof(maskValue));
            vif1_regs.mask = maskValue;
            pos += 4;
            continue;
        }
        else if (opcode == VIF_STROW)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.row, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_STCOL)
        {
            if (pos + 16 > sizeBytes)
                break;
            std::memcpy(vif1_regs.col, data + pos, 16);
            pos += 16;
            continue;
        }
        else if (opcode == VIF_MPG)
        {
            uint32_t destAddr = (uint32_t)imm * 8u;
            const uint32_t instructionCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);
            const uint32_t mpgBytes = instructionCount * 8u;
            if (m_vu1Code && destAddr < PS2_VU1_CODE_SIZE && mpgBytes > 0)
            {
                uint32_t copyBytes = mpgBytes;
                if (destAddr + copyBytes > PS2_VU1_CODE_SIZE)
                    copyBytes = PS2_VU1_CODE_SIZE - destAddr;
                if (pos + copyBytes <= sizeBytes)
                {
                    std::memcpy(m_vu1Code + destAddr, data + pos, copyBytes);
                    markVU1CodeModified();
                }
            }
            pos += mpgBytes;
            if (pos > sizeBytes)
                break;
            continue;
        }
        else if (opcode == VIF_DIRECT || opcode == VIF_DIRECTHL)
        {
            uint32_t qwCount = imm;
            if (qwCount == 0)
                qwCount = 65536;
            const uint32_t availableQw = (sizeBytes - pos) / 16u;
            const bool truncated = qwCount > availableQw;
            if (qwCount > availableQw)
                qwCount = availableQw;

            if (qwCount > 0)
            {
                const bool directHl = (opcode == VIF_DIRECTHL);
                submitGifPacket(GifPathId::Path2, data + pos, qwCount * 16, true, directHl);

                const uint32_t pendingImageQw = pendingGifImageQwc(data + pos, qwCount * 16u);
                if (pendingImageQw != 0u)
                {
                    m_vif1PendingPath2ImageQwc = pendingImageQw;
                    m_vif1PendingPath2DirectHl = directHl;
                }
            }

            pos += qwCount * 16;
            if (truncated)
            {
                pos = sizeBytes;
                break;
            }
            continue;
        }
        else if ((opcode & 0x60) == 0x60)
        {
            uint8_t vn = (opcode >> 2) & 0x3;
            uint8_t vl = opcode & 0x3;
            const bool maskEnable = (opcode & 0x10u) != 0u;
            int components = vn + 1;
            int bitsPerComponent = 32;
            switch (vl)
            {
            case 0:
                bitsPerComponent = 32;
                break;
            case 1:
                bitsPerComponent = 16;
                break;
            case 2:
                bitsPerComponent = 8;
                break;
            case 3:
                bitsPerComponent = (vn == 3) ? 4 : 16;
                break;
            default:
                break;
            }
            int bitsPerVector = (vl == 3 && vn == 3) ? 16 : (components * bitsPerComponent);
            uint32_t bytesPerVector = (bitsPerVector + 7) / 8;
            // UNPACK semantics: NUM is 8-bit and NUM==0 means 256 vectors (writes).
            const uint32_t writeVectorCount = (num == 0u) ? 256u : static_cast<uint32_t>(num);

            // STCYCL controls write cycles for UNPACK.
            uint32_t cl = vif1_regs.cycle & 0xFFu;
            uint32_t wl = (vif1_regs.cycle >> 8) & 0xFFu;
            if (cl == 0u)
                cl = 1u;
            if (wl == 0u)
                wl = 1u;

            uint32_t sourceVectorCount = writeVectorCount;
            if (cl < wl)
            {
                const uint32_t fullBlocks = writeVectorCount / wl;
                uint32_t remainder = writeVectorCount % wl;
                if (remainder > cl)
                    remainder = cl;
                sourceVectorCount = fullBlocks * cl + remainder;
            }

            uint32_t totalBytes = sourceVectorCount * bytesPerVector;
            totalBytes = (totalBytes + 3) & ~3u;

            uint32_t vuAddr = (uint32_t)imm & 0x3FFu;
            if ((imm & 0x8000u) != 0u)
                vuAddr = (vuAddr + (vif1_regs.tops & 0x3FFu)) & 0x3FFu;

            const bool zeroExtend = (imm & 0x4000u) != 0u;

            if (!legacyVif1UnpackRequested())
            {
                // Source size as PCSX2: fill mode reads CL vectors per WL block (WL = 0 means 256).
                const bool linear = (vif1_regs.cycle & 0xFFFFu) == 0u;
                const uint32_t pcl = linear ? 1u : (vif1_regs.cycle & 0xFFu);
                const uint32_t pwl = linear ? 1u : (((vif1_regs.cycle >> 8) & 0xFFu) ? ((vif1_regs.cycle >> 8) & 0xFFu) : 256u);
                const uint32_t srcVectors = (pcl < pwl) ? pcl * (writeVectorCount / pwl) + std::min(writeVectorCount % pwl, pcl)
                                                        : writeVectorCount;
                const uint32_t srcBytes = (srcVectors * kVifUnpackSourceBytes[opcode & 0xFu] + 3u) & ~3u;
                if (m_vu1Data && pos + srcBytes <= sizeBytes)
                    vif1UnpackPcsx2(m_vu1Data, PS2_VU1_DATA_SIZE, vif1_regs, cmd, data + pos, srcBytes);
                pos += srcBytes;
                if (pos > sizeBytes)
                    break;
                continue;
            }

            if (m_vu1Data && totalBytes > 0 && pos + totalBytes <= sizeBytes)
            {
                const uint8_t *srcBase = data + pos;
                uint32_t srcIndex = 0u;
                for (uint32_t writeIndex = 0; writeIndex < writeVectorCount; ++writeIndex)
                {
                    const uint32_t cyclePos = writeIndex % wl;
                    const bool sourceAvailable = (cl >= wl) || (cyclePos < cl);

                    uint32_t destVec = 0;
                    if (cl >= wl)
                    {
                        destVec = (vuAddr + (writeIndex / wl) * cl + cyclePos) & 0x3FFu;
                    }
                    else
                    {
                        destVec = (vuAddr + writeIndex) & 0x3FFu;
                    }

                    uint32_t destOff = destVec * 16u;
                    if (destOff + 16u > PS2_VU1_DATA_SIZE)
                    {
                        if (sourceAvailable && srcIndex < sourceVectorCount)
                            ++srcIndex;
                        continue;
                    }

                    uint32_t lanes[4] = {0u, 0u, 0u, 0u};
                    std::memcpy(lanes, m_vu1Data + destOff, sizeof(lanes));
                    uint32_t decompressed[4] = {lanes[0], lanes[1], lanes[2], lanes[3]};
                    bool decoded = false;

                    const uint8_t *srcVec = nullptr;
                    if (sourceAvailable && srcIndex < sourceVectorCount)
                    {
                        srcVec = srcBase + srcIndex * bytesPerVector;
                        ++srcIndex;
                        decoded = true;
                    }

                    auto extend16 = [&](uint16_t raw) -> uint32_t
                    {
                        if (zeroExtend)
                            return static_cast<uint32_t>(raw);
                        return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(raw)));
                    };

                    auto extend8 = [&](uint8_t raw) -> uint32_t
                    {
                        if (zeroExtend)
                            return static_cast<uint32_t>(raw);
                        return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(raw)));
                    };

                    bool handledFormat = true;
                    if (!decoded)
                    {
                        handledFormat = false;
                    }
                    else if (vl == 0u)
                    {
                        if (components == 1)
                        {
                            uint32_t scalar = 0;
                            std::memcpy(&scalar, srcVec, sizeof(scalar));
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                uint32_t scalar = 0;
                                std::memcpy(&scalar, srcVec + c * 4u, sizeof(scalar));
                                decompressed[c] = scalar;
                            }
                        }
                    }
                    else if (vl == 1u)
                    {
                        if (components == 1)
                        {
                            uint16_t raw = 0;
                            std::memcpy(&raw, srcVec, sizeof(raw));
                            const uint32_t scalar = extend16(raw);
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                uint16_t raw = 0;
                                std::memcpy(&raw, srcVec + c * 2u, sizeof(raw));
                                decompressed[c] = extend16(raw);
                            }
                        }
                    }
                    else if (vl == 2u)
                    {
                        if (components == 1)
                        {
                            const uint32_t scalar = extend8(srcVec[0]);
                            decompressed[0] = scalar;
                            decompressed[1] = scalar;
                            decompressed[2] = scalar;
                            decompressed[3] = scalar;
                        }
                        else
                        {
                            const uint32_t limit = (components > 4) ? 4u : static_cast<uint32_t>(components);
                            for (uint32_t c = 0; c < limit; ++c)
                            {
                                decompressed[c] = extend8(srcVec[c]);
                            }
                        }
                    }
                    else if (vl == 3u && vn == 3u)
                    {
                        // V4-5: packed color-like format in a single 16-bit value.
                        uint16_t packed = 0;
                        std::memcpy(&packed, srcVec, sizeof(packed));
                        decompressed[0] = packed & 0x1Fu;
                        decompressed[1] = (packed >> 5) & 0x1Fu;
                        decompressed[2] = (packed >> 10) & 0x1Fu;
                        decompressed[3] = (packed >> 15) & 0x01u;
                    }
                    else
                    {
                        handledFormat = false;
                    }

                    // Unknown compressed format fallback: preserve legacy raw-copy behavior.
                    if (!handledFormat && decoded && !maskEnable && (vif1_regs.mode == 0u || vif1_regs.mode == 3u))
                    {
                        uint32_t copyBytes = (bytesPerVector < 16u) ? bytesPerVector : 16u;
                        std::memcpy(m_vu1Data + destOff, srcVec, copyBytes);
                        continue;
                    }

                    const bool canAdd = (vl != 3u || vn != 3u);
                    const uint32_t mode = vif1_regs.mode & 3u;
                    const uint32_t colIdx = (cyclePos > 3u) ? 3u : cyclePos;
                    const uint32_t maskCycle = (cyclePos > 3u) ? 3u : cyclePos;

                    for (uint32_t field = 0u; field < 4u; ++field)
                    {
                        uint32_t maskSpec = 0u;
                        if (maskEnable)
                        {
                            const uint32_t shift = ((maskCycle * 4u) + field) * 2u;
                            maskSpec = (vif1_regs.mask >> shift) & 0x3u;
                        }

                        // In fill-write cycles with suspended source reads, treat raw-data selections as row-fill.
                        if (!decoded && maskSpec == 0u)
                            maskSpec = 1u;

                        uint32_t writeVal = lanes[field];
                        if (maskSpec == 0u)
                        {
                            if (handledFormat)
                            {
                                writeVal = decompressed[field];
                                if (canAdd && (mode == 1u || mode == 2u))
                                {
                                    writeVal = writeVal + vif1_regs.row[field];
                                    if (mode == 2u)
                                        vif1_regs.row[field] = writeVal;
                                }
                            }
                        }
                        else if (maskSpec == 1u)
                        {
                            writeVal = vif1_regs.row[field];
                        }
                        else if (maskSpec == 2u)
                        {
                            writeVal = vif1_regs.col[colIdx];
                        }
                        else
                        {
                            continue; // write-protect
                        }

                        lanes[field] = writeVal;
                    }

                    std::memcpy(m_vu1Data + destOff, lanes, sizeof(lanes));
                }
            }
            pos += totalBytes;

            if (pos > sizeBytes)
                break;
            continue;
        }
        else
        {
            continue;
        }
    }
}

// ---- threaded VIF1/VU1 (MTVU) ----

void PS2Memory::processVIF1Data(const uint8_t *data, uint32_t sizeBytes)
{
    if (!m_vif1Threaded.load(std::memory_order_acquire) || onVif1Thread() || !data || sizeBytes == 0u)
    {
        processVIF1DataNow(data, sizeBytes);
        return;
    }
    Vif1QueueItem item;
    item.kind = Vif1QueueItem::Kind::Vif1Data;
    // Copy now: the game reuses its DMA buffers once the transfer is done.
    item.data = takeVif1Buffer();
    item.data.assign(data, data + sizeBytes);
    enqueueVif1Item(std::move(item));
}

std::vector<uint8_t> PS2Memory::takeVif1Buffer()
{
    std::lock_guard<std::mutex> lock(m_vif1QueueMutex);
    if (m_vif1FreeBuffers.empty())
        return {};
    std::vector<uint8_t> buffer = std::move(m_vif1FreeBuffers.back());
    m_vif1FreeBuffers.pop_back();
    buffer.clear();
    return buffer;
}

void PS2Memory::enqueueVif1Item(Vif1QueueItem &&item)
{
    const size_t bytes = item.data.size() + 16u;
    constexpr size_t kMaxQueuedBytes = 16u * 1024u * 1024u;
    std::unique_lock<std::mutex> lock(m_vif1QueueMutex);
    m_vif1IdleCv.wait(lock, [&]
                      { return m_vif1QueuedBytes < kMaxQueuedBytes || m_vif1WorkerStop; });
    m_vif1QueuedBytes += bytes;
    m_vif1Queue.push_back(std::move(item));
    ++m_vif1Submitted;
    lock.unlock();
    m_vif1QueueCv.notify_one();
}

void PS2Memory::setVif1Threaded(bool enabled, std::function<void()> onWorkerStart)
{
    if (enabled == m_vif1Threaded.load(std::memory_order_acquire))
        return;
    if (enabled)
    {
        {
            std::lock_guard<std::mutex> lock(m_vif1QueueMutex);
            m_vif1WorkerStop = false;
        }
        m_vif1Worker = std::thread([this, start = std::move(onWorkerStart)]() mutable
                                   { vif1WorkerLoop(std::move(start)); });
        m_vif1WorkerId = m_vif1Worker.get_id(); // published by the release store below
        m_vif1Threaded.store(true, std::memory_order_release);
        return;
    }
    waitVif1Idle();
    m_vif1Threaded.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(m_vif1QueueMutex);
        m_vif1WorkerStop = true;
    }
    m_vif1QueueCv.notify_all();
    if (m_vif1Worker.joinable())
        m_vif1Worker.join();
    m_vif1WorkerId = std::thread::id{};
}

namespace
{
    // PS2X_SYNC_STATS=1: how often and how long the EE waits for the VU thread.
    struct SyncStats
    {
        bool enabled = []
        {
            const char *v = std::getenv("PS2X_SYNC_STATS");
            return v && *v && *v != '0';
        }();
        std::atomic<uint64_t> calls{0}, waits{0}, waitNs{0};
        std::atomic<int64_t> lastReport{0};
    };
    SyncStats g_vif1Sync;
}

void PS2Memory::waitVif1Idle(uint32_t reason) const
{
    if (!m_vif1Threaded.load(std::memory_order_acquire) || onVif1Thread())
        return;
    const auto start = std::chrono::steady_clock::now();
    bool waited = false;
    {
        std::unique_lock<std::mutex> lock(m_vif1QueueMutex);
        const uint64_t target = m_vif1Submitted;
        waited = m_vif1Completed < target;
        m_vif1IdleCv.wait(lock, [&]
                          { return m_vif1Completed >= target || m_vif1WorkerStop; });
    }
    if (!g_vif1Sync.enabled)
        return;
    ++g_vif1Sync.calls;
    const auto now = std::chrono::steady_clock::now();
    if (waited)
    {
        ++g_vif1Sync.waits;
        g_vif1Sync.waitNs += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count());
    }
    const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    int64_t last = g_vif1Sync.lastReport.load();
    if (nowMs - last >= 2000 && g_vif1Sync.lastReport.compare_exchange_strong(last, nowMs))
    {
        std::cerr << "[sync:vif1] calls=" << g_vif1Sync.calls.exchange(0) << " waits=" << g_vif1Sync.waits.exchange(0)
                  << " wait_ms=" << g_vif1Sync.waitNs.exchange(0) / 1000000u << " last_reason=0x" << std::hex << reason
                  << std::dec << std::endl;
    }
}

void PS2Memory::vif1WorkerLoop(std::function<void()> onWorkerStart)
{
    if (onWorkerStart)
        onWorkerStart();
    for (;;)
    {
        Vif1QueueItem item;
        {
            std::unique_lock<std::mutex> lock(m_vif1QueueMutex);
            m_vif1QueueCv.wait(lock, [&]
                               { return !m_vif1Queue.empty() || m_vif1WorkerStop; });
            if (m_vif1Queue.empty())
                break;
            item = std::move(m_vif1Queue.front());
            m_vif1Queue.pop_front();
        }
        switch (item.kind)
        {
        case Vif1QueueItem::Kind::Vif1Data:
            processVIF1DataNow(item.data.data(), static_cast<uint32_t>(item.data.size()));
            break;
        case Vif1QueueItem::Kind::GifPacket:
            submitGifPacket(item.path, item.data.data(), static_cast<uint32_t>(item.data.size()),
                            item.drainImmediately, item.path2DirectHl);
            break;
        case Vif1QueueItem::Kind::GifDrain:
            if (m_gifArbiter)
            {
                std::lock_guard<std::recursive_mutex> gifLock(m_gifMutex);
                m_gifArbiter->drain();
            }
            break;
        }
        {
            std::lock_guard<std::mutex> lock(m_vif1QueueMutex);
            m_vif1QueuedBytes -= item.data.size() + 16u;
            ++m_vif1Completed;
            if (m_vif1FreeBuffers.size() < 256u && item.data.capacity() != 0u)
            {
                item.data.clear();
                m_vif1FreeBuffers.push_back(std::move(item.data));
            }
        }
        m_vif1IdleCv.notify_all();
    }
}
