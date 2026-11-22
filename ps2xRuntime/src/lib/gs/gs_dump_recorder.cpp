#include "runtime/gs/gs_dump_recorder.h"

#include "runtime/ps2_memory.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

namespace
{
    // PCSX2 GSdx dump layout (see paraLLEl-GS dump/gs_dump_generator.cpp).
    constexpr uint32_t kStateVersion = 9u;
    constexpr uint32_t kStateVersionMin = 8u;
    constexpr uint8_t kPacketTransfer = 0u;
    constexpr uint8_t kPacketVsync = 1u;
    constexpr uint8_t kPacketPrivRegisters = 3u;
    constexpr size_t kPrivRegisterStateSize = 0x2000u; // 2 x 0x200 qwords
    constexpr uint64_t kGifRegAD = 0xEu;

    struct DumpHeader
    {
        uint32_t version;
        uint32_t stateSize;
        uint32_t serialOffset;
        uint32_t serialSize;
        uint32_t crc;
        uint32_t screenshotWidth;
        uint32_t screenshotHeight;
        uint32_t screenshotOffset;
        uint32_t screenshotSize;
    };

    uint64_t adTagLo(uint32_t nloop, bool eop)
    {
        // NLOOP, EOP, PRE=0, FLG=PACKED, NREG=1
        return static_cast<uint64_t>(nloop & 0x7FFFu) | (eop ? (1ull << 15) : 0ull) | (1ull << 60);
    }
}

GSDumpRecorder::~GSDumpRecorder()
{
    close();
}

void GSDumpRecorder::close()
{
    if (m_file)
    {
        std::fclose(m_file);
        m_file = nullptr;
    }
}

void GSDumpRecorder::writeBytes(const void *data, size_t size)
{
    if (m_file && std::fwrite(data, 1, size, m_file) != size)
    {
        std::cerr << "[gs:dump] write failed, recording stopped" << std::endl;
        close();
    }
}

void GSDumpRecorder::startFromEnvironment(const uint8_t *vram, uint32_t vramSize)
{
    const char *path = std::getenv("PS2X_GS_DUMP");
    if (!path || !*path || !vram)
        return;
    std::lock_guard<std::mutex> lock(m_mutex);
    close();
    m_file = std::fopen(path, "wb");
    if (!m_file)
    {
        std::cerr << "[gs:dump] cannot open " << path << std::endl;
        return;
    }
    m_path = path;
    m_vsyncCount = 0u;
    m_vramSnapshots.clear();
    if (const char *at = std::getenv("PS2X_GS_DUMP_VRAM_AT"))
    {
        for (const char *p = at; *p;)
        {
            char *end = nullptr;
            const unsigned long value = std::strtoul(p, &end, 10);
            if (end == p)
                break;
            m_vramSnapshots.insert(static_cast<uint32_t>(value));
            p = (*end == ',') ? end + 1 : end;
        }
    }
    m_framesLeft = 600u;
    if (const char *frames = std::getenv("PS2X_GS_DUMP_FRAMES"))
        m_framesLeft = static_cast<uint32_t>(std::max(1L, std::strtol(frames, nullptr, 10)));

    writeU32(UINT32_MAX); // fake CRC
    DumpHeader header{};
    header.version = kStateVersion;
    header.stateSize = vramSize + (18u + 12u * 2u) * 8u + 7u * 4u + 4u * 5u * 4u + 4u;
    writeU32(sizeof(header));
    writeBytes(&header, sizeof(header));

    // Register state: defaults, except PRMODECONT.AC = 1 (use PRIM).
    writeU32(kStateVersionMin);
    const uint64_t globals[15] = {
        0ull /*PRIM*/, 1ull /*PRMODECONT*/, 0ull /*TEXCLUT*/, 0ull /*SCANMSK*/, 0ull /*TEXA*/,
        0ull /*FOGCOL*/, 0ull /*DIMX*/, 0ull /*DTHE*/, 1ull /*COLCLAMP*/, 0ull /*PABE*/,
        0ull /*BITBLTBUF*/, 0ull /*TRXDIR*/, 0ull /*TRXPOS*/, 0ull /*TRXREG*/, 0ull /*dummy*/};
    writeBytes(globals, sizeof(globals));
    const uint64_t context[12] = {}; // XYOFFSET TEX0 TEX1 CLAMP MIPTBP1 MIPTBP2 SCISSOR ALPHA TEST FBA FRAME ZBUF
    writeBytes(context, sizeof(context));
    writeBytes(context, sizeof(context));
    writeU64(0ull);       // RGBAQ
    writeU64(0ull);       // ST
    writeU32(0u);         // UV
    writeU32(0u);         // FOG
    writeU64(0ull);       // XYZ
    writeU32(UINT32_MAX); // GIFReg dummies
    writeU32(UINT32_MAX);
    writeU32(0u); // transfer X/Y
    writeU32(0u);
    writeBytes(vram, vramSize);
    for (int path = 0; path < 4; ++path)
    {
        const uint64_t tag[2] = {};
        writeBytes(tag, sizeof(tag));
        writeU32(0u);
    }
    const float internalQ = 1.0f;
    writeBytes(&internalQ, sizeof(internalQ));
    const std::vector<uint8_t> priv(kPrivRegisterStateSize, 0u);
    writeBytes(priv.data(), priv.size());
    std::cerr << "[gs:dump] recording to " << path << " (" << m_framesLeft << " frames)" << std::endl;
}

void GSDumpRecorder::writeTransferHeader(uint8_t path, uint32_t sizeBytes)
{
    writeU8(kPacketTransfer);
    writeU8(path);
    writeU32(sizeBytes);
}

void GSDumpRecorder::recordTransfer(uint8_t path, const uint8_t *data, uint32_t sizeBytes)
{
    if (!m_file || !data || sizeBytes == 0u)
        return;
    std::lock_guard<std::mutex> lock(m_mutex);
    const uint32_t aligned = sizeBytes & ~15u;
    if (aligned == 0u)
        return;
    writeTransferHeader(path, aligned);
    writeBytes(data, aligned);
}

void GSDumpRecorder::recordRegister(uint8_t regAddr, uint64_t value)
{
    if (!m_file)
        return;
    std::lock_guard<std::mutex> lock(m_mutex);
    const uint64_t packet[4] = {adTagLo(1u, true), kGifRegAD, value, static_cast<uint64_t>(regAddr)};
    writeTransferHeader(3u, sizeof(packet));
    writeBytes(packet, sizeof(packet));
}

void GSDumpRecorder::recordImageUpload(uint64_t bitbltbuf, uint64_t trxpos, uint64_t trxreg, uint64_t trxdir,
                                       const uint8_t *data, uint32_t sizeBytes)
{
    if (!m_file || !data)
        return;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const uint64_t setup[10] = {adTagLo(4u, false), kGifRegAD,
                                    bitbltbuf, 0x50ull, trxpos, 0x51ull, trxreg, 0x52ull, trxdir, 0x53ull};
        writeTransferHeader(3u, sizeof(setup));
        writeBytes(setup, sizeof(setup));

        const uint32_t aligned = (sizeBytes + 15u) & ~15u;
        constexpr uint32_t kChunk = 32u * 1024u * 16u;
        for (uint32_t offset = 0u; offset < aligned;)
        {
            const uint32_t chunk = std::min(aligned - offset, kChunk);
            // IMAGE mode: FLG = 2
            const uint64_t tag[2] = {static_cast<uint64_t>(chunk / 16u) |
                                         (offset + chunk == aligned ? (1ull << 15) : 0ull) | (2ull << 58),
                                     0ull};
            writeTransferHeader(3u, chunk + 16u);
            writeBytes(tag, sizeof(tag));
            const uint32_t available = offset < sizeBytes ? std::min(chunk, sizeBytes - offset) : 0u;
            writeBytes(data + offset, available);
            static const uint8_t zeros[16] = {};
            for (uint32_t pad = available; pad < chunk;)
            {
                const uint32_t n = std::min<uint32_t>(16u, chunk - pad);
                writeBytes(zeros, n);
                pad += n;
            }
            offset += chunk;
        }
    }
}

void GSDumpRecorder::recordVsync(const GSRegisters &regs, uint32_t field)
{
    if (!m_file)
        return;
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<uint8_t> priv(kPrivRegisterStateSize, 0u);
    const uint64_t low[15] = {regs.pmode, regs.smode1, regs.smode2, regs.srfsh, regs.synch1,
                              regs.synch2, regs.syncv, regs.dispfb1, regs.display1, regs.dispfb2,
                              regs.display2, regs.extbuf, regs.extdata, regs.extwrite, regs.bgcolor};
    for (size_t i = 0; i < 15u; ++i)
        std::memcpy(priv.data() + i * 16u, &low[i], 8u);
    const uint64_t csr = regs.csr.load(std::memory_order_relaxed);
    std::memcpy(priv.data() + 0x1000u, &csr, 8u);
    std::memcpy(priv.data() + 0x1010u, &regs.imr, 8u);
    std::memcpy(priv.data() + 0x1040u, &regs.busdir, 8u);
    std::memcpy(priv.data() + 0x1080u, &regs.siglblid, 8u);
    writeU8(kPacketPrivRegisters);
    writeBytes(priv.data(), priv.size());
    writeU8(kPacketVsync);
    writeU8(static_cast<uint8_t>(field & 1u));
    ++m_vsyncCount;
    if (m_framesLeft > 0u && --m_framesLeft == 0u)
    {
        std::cerr << "[gs:dump] frame budget reached, recording stopped" << std::endl;
        close();
    }
}

void GSDumpRecorder::snapshotVramIfRequested(const uint8_t *vram, uint32_t vramSize)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!vram || !m_vramSnapshots.count(m_vsyncCount))
        return;
    m_vramSnapshots.erase(m_vsyncCount);
    const std::string out = m_path + "_vram_" + std::to_string(m_vsyncCount) + ".bin";
    if (std::FILE *file = std::fopen(out.c_str(), "wb"))
    {
        std::fwrite(vram, 1, vramSize, file);
        std::fclose(file);
        std::cerr << "[gs:dump] wrote " << out << std::endl;
    }
}
