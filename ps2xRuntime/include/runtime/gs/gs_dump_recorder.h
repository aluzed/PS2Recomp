#pragma once

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <set>
#include <string>

struct GSRegisters;

// Records everything the GS frontend receives into a PCSX2-format GS dump
// (state version 9), so a frame can be replayed in another GS implementation
// (PCSX2, paraLLEl-GS's parallel-gs-replayer) and compared pass by pass.
//
// Enabled by environment variables read at GS::init:
//   PS2X_GS_DUMP=<path>          output file
//   PS2X_GS_DUMP_FRAMES=<n>      stop after n presented frames (default 600)
//   PS2X_GS_DUMP_VRAM_AT=<a,b>   also write raw local memory after those vsyncs
//                                (1-based) to <path>_vram_<n>.bin
//
// Recording starts at init with the current local memory and a default
// register state, so it captures from boot. Register writes performed directly
// by HLE stubs are recorded as A+D transfers; privileged registers are written
// before every vsync marker.
class GSDumpRecorder
{
public:
    ~GSDumpRecorder();

    void startFromEnvironment(const uint8_t *vram, uint32_t vramSize);
    bool active() const { return m_file != nullptr; }

    void recordTransfer(uint8_t path, const uint8_t *data, uint32_t sizeBytes);
    void recordRegister(uint8_t regAddr, uint64_t value);
    void recordImageUpload(uint64_t bitbltbuf, uint64_t trxpos, uint64_t trxreg, uint64_t trxdir,
                           const uint8_t *data, uint32_t sizeBytes);
    void recordVsync(const GSRegisters &privRegs, uint32_t field);
    // Call after the backend has flushed the frame that ended at the last vsync.
    void snapshotVramIfRequested(const uint8_t *vram, uint32_t vramSize);

private:
    void writeBytes(const void *data, size_t size);
    void writeU8(uint8_t value) { writeBytes(&value, 1); }
    void writeU32(uint32_t value) { writeBytes(&value, 4); }
    void writeU64(uint64_t value) { writeBytes(&value, 8); }
    void writeTransferHeader(uint8_t path, uint32_t sizeBytes);
    void close();

    std::FILE *m_file = nullptr;
    std::mutex m_mutex;
    uint32_t m_framesLeft = 0;
    uint32_t m_vsyncCount = 0;
    std::set<uint32_t> m_vramSnapshots;
    std::string m_path;
};
