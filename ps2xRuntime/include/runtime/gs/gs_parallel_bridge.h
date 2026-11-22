#pragma once

#include <cstdint>
#include <memory>
#include <vector>

struct GSRegisters;

// Optional hardware GS: forwards the GS input stream to paraLLEl-GS
// (Vulkan compute, https://github.com/Arntzen-Software/parallel-gs) and reads
// back the scanout image for presentation. The runtime's GS frontend keeps
// decoding packets for its own state (CSR, transfers); only rasterization and
// scanout move to the GPU.
//
// Built when the runtime is configured with PS2X_PARALLEL_GS_DIR. Selected at
// runtime unless PS2X_GS_BACKEND=cpu.
class GSParallelBridge
{
public:
    GSParallelBridge();
    ~GSParallelBridge();

    // Creates a headless Vulkan device and the GS. Returns false (and logs)
    // when Vulkan is unavailable; the caller then keeps the CPU backend.
    bool initialize(const uint8_t *initialVram, uint32_t vramSize);

    void transfer(uint32_t path, const uint8_t *data, uint32_t sizeBytes);
    void writeRegister(uint8_t regAddr, uint64_t value);
    void imageUpload(uint64_t bitbltbuf, uint64_t trxpos, uint64_t trxreg, uint64_t trxdir,
                     const uint8_t *data, uint32_t sizeBytes);

    // Scans out the current frame. Pixels are tightly packed RGBA8.
    bool vsync(const GSRegisters &privRegs, uint32_t field,
               std::vector<uint8_t> &pixels, uint32_t &width, uint32_t &height);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
