#include "runtime/gs/gs_parallel_bridge.h"

#include "runtime/ps2_memory.h"

#if !defined(PS2X_ENABLE_PARALLEL_GS)

// Built without paraLLEl-GS: the bridge never initializes and the GS frontend
// keeps the CPU backend.
struct GSParallelBridge::Impl
{
};
GSParallelBridge::GSParallelBridge() = default;
GSParallelBridge::~GSParallelBridge() = default;
bool GSParallelBridge::initialize(const uint8_t *, uint32_t) { return false; }
void GSParallelBridge::transfer(uint32_t, const uint8_t *, uint32_t) {}
void GSParallelBridge::writeRegister(uint8_t, uint64_t) {}
void GSParallelBridge::imageUpload(uint64_t, uint64_t, uint64_t, uint64_t, const uint8_t *, uint32_t) {}
bool GSParallelBridge::vsync(const GSRegisters &, uint32_t, std::vector<uint8_t> &, uint32_t &, uint32_t &) { return false; }

#else

#include "context.hpp"
#include "device.hpp"
#include "gs_interface.hpp"
#include "thread_id.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace
{
    constexpr uint64_t kGifRegAD = 0xEu;

    // Granite looks up a per-thread index for command pools. The GS is driven
    // from the guest's game thread as well as the host thread that created it
    // (one at a time), so every caller registers as index 0.
    void ensureGraniteThread()
    {
        static thread_local bool registered = false;
        if (!registered)
        {
            Util::register_thread_index(0);
            registered = true;
        }
    }

    uint64_t adTagLo(uint32_t nloop, bool eop)
    {
        // NLOOP, EOP, FLG = PACKED, NREG = 1
        return static_cast<uint64_t>(nloop & 0x7FFFu) | (eop ? (1ull << 15) : 0ull) | (1ull << 60);
    }
}

struct GSParallelBridge::Impl
{
    Vulkan::Context context;
    Vulkan::Device device;
    ParallelGS::GSInterface gs;
    bool ready = false;
};

GSParallelBridge::GSParallelBridge() = default;
GSParallelBridge::~GSParallelBridge() = default;

bool GSParallelBridge::initialize(const uint8_t *initialVram, uint32_t vramSize)
{
    ensureGraniteThread();
    if (!Vulkan::Context::init_loader(nullptr))
    {
        std::cerr << "[gs:parallel] Vulkan loader not available" << std::endl;
        return false;
    }
    m_impl = std::make_unique<Impl>();
    m_impl->context.set_num_thread_indices(1);
    if (!m_impl->context.init_instance_and_device(nullptr, 0, nullptr, 0,
                                                   Vulkan::CONTEXT_CREATION_ENABLE_PUSH_DESCRIPTOR_BIT |
                                                       Vulkan::CONTEXT_CREATION_ENABLE_DESCRIPTOR_HEAP_BIT |
                                                       Vulkan::CONTEXT_CREATION_ENABLE_DESCRIPTOR_BUFFER_BIT))
    {
        std::cerr << "[gs:parallel] cannot create a Vulkan device" << std::endl;
        m_impl.reset();
        return false;
    }
    m_impl->device.set_context(m_impl->context);
    m_impl->device.init_frame_contexts(4);

    ParallelGS::GSOptions options{};
    options.vram_size = vramSize;
    if (!m_impl->gs.init(&m_impl->device, options))
    {
        std::cerr << "[gs:parallel] GSInterface init failed" << std::endl;
        m_impl.reset();
        return false;
    }
    if (initialVram)
    {
        void *dst = m_impl->gs.map_vram_write(0, vramSize);
        std::memcpy(dst, initialVram, vramSize);
        m_impl->gs.end_vram_write(0, vramSize);
    }
    // Same defaults as a fresh GS: PRMODECONT.AC = 1, COLCLAMP = 1.
    writeRegister(0x1A, 1u);
    writeRegister(0x46, 1u);
    m_impl->ready = true;
    std::cerr << "[gs:parallel] hardware GS active ("
              << m_impl->device.get_gpu_properties().deviceName << ")" << std::endl;
    return true;
}

void GSParallelBridge::transfer(uint32_t path, const uint8_t *data, uint32_t sizeBytes)
{
    ensureGraniteThread();
    const uint32_t aligned = sizeBytes & ~15u;
    if (!m_impl || !m_impl->ready || !data || aligned == 0u)
        return;
    m_impl->gs.gif_transfer(path, data, aligned);
}

void GSParallelBridge::writeRegister(uint8_t regAddr, uint64_t value)
{
    ensureGraniteThread();
    if (!m_impl)
        return;
    const uint64_t packet[4] = {adTagLo(1u, true), kGifRegAD, value, static_cast<uint64_t>(regAddr)};
    m_impl->gs.gif_transfer(3, packet, sizeof(packet));
}

void GSParallelBridge::imageUpload(uint64_t bitbltbuf, uint64_t trxpos, uint64_t trxreg, uint64_t trxdir,
                                   const uint8_t *data, uint32_t sizeBytes)
{
    ensureGraniteThread();
    if (!m_impl || !m_impl->ready || !data)
        return;
    const uint64_t setup[10] = {adTagLo(4u, false), kGifRegAD,
                                bitbltbuf, 0x50ull, trxpos, 0x51ull, trxreg, 0x52ull, trxdir, 0x53ull};
    m_impl->gs.gif_transfer(3, setup, sizeof(setup));

    const uint32_t aligned = (sizeBytes + 15u) & ~15u;
    std::vector<uint8_t> packet;
    constexpr uint32_t kChunk = 32u * 1024u * 16u;
    for (uint32_t offset = 0u; offset < aligned;)
    {
        const uint32_t chunk = std::min(aligned - offset, kChunk);
        packet.assign(16u + chunk, 0u);
        const uint64_t tagLo = static_cast<uint64_t>(chunk / 16u) |
                               (offset + chunk == aligned ? (1ull << 15) : 0ull) | (2ull << 58); // IMAGE
        std::memcpy(packet.data(), &tagLo, sizeof(tagLo));
        const uint32_t available = offset < sizeBytes ? std::min(chunk, sizeBytes - offset) : 0u;
        std::memcpy(packet.data() + 16u, data + offset, available);
        m_impl->gs.gif_transfer(3, packet.data(), packet.size());
        offset += chunk;
    }
}

bool GSParallelBridge::vsync(const GSRegisters &regs, uint32_t field,
                             std::vector<uint8_t> &pixels, uint32_t &width, uint32_t &height)
{
    ensureGraniteThread();
    if (!m_impl || !m_impl->ready)
        return false;

    // Privileged registers: same 16-byte stride layout as the GS.
    auto &priv = m_impl->gs.get_priv_register_state();
    const uint64_t low[15] = {regs.pmode, regs.smode1, regs.smode2, regs.srfsh, regs.synch1,
                              regs.synch2, regs.syncv, regs.dispfb1, regs.display1, regs.dispfb2,
                              regs.display2, regs.extbuf, regs.extdata, regs.extwrite, regs.bgcolor};
    for (size_t i = 0; i < 15u; ++i)
        priv.qwords_lo[i * 2u] = low[i];

    ParallelGS::VSyncInfo info{};
    info.phase = field & 1u;
    info.dst_layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    info.dst_stage = VK_PIPELINE_STAGE_2_COPY_BIT;
    info.dst_access = VK_ACCESS_2_TRANSFER_READ_BIT;
    info.force_progressive = true;
    info.anti_blur = true;
    info.adapt_to_internal_horizontal_resolution = true;

    m_impl->gs.flush();
    ParallelGS::ScanoutResult result = m_impl->gs.vsync(info);
    {
        static uint32_t s_calls = 0u, s_images = 0u;
        ++s_calls;
        if (result.image)
            ++s_images;
        if ((s_calls % 120u) == 0u)
            std::cerr << "[gs:parallel] vsync calls=" << s_calls << " images=" << s_images
                      << " last=" << (result.image ? result.image->get_width() : 0u) << "x"
                      << (result.image ? result.image->get_height() : 0u) << " pmode=0x" << std::hex
                      << regs.pmode << " smode2=0x" << regs.smode2 << std::dec << std::endl;
    }
    m_impl->device.next_frame_context();
    if (!result.image)
    {
        static uint32_t s_empty = 0u;
        if (s_empty++ < 3u)
            std::cerr << "[gs:parallel] vsync produced no image (pmode=0x" << std::hex << regs.pmode
                      << " dispfb1=0x" << regs.dispfb1 << " display1=0x" << regs.display1 << std::dec << ")" << std::endl;
        return false;
    }

    width = result.image->get_width();
    height = result.image->get_height();
    const VkDeviceSize bytes = VkDeviceSize(width) * height * 4u;

    Vulkan::BufferCreateInfo bufferInfo{};
    bufferInfo.size = bytes;
    bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bufferInfo.domain = Vulkan::BufferDomain::CachedHost;
    auto buffer = m_impl->device.create_buffer(bufferInfo);

    auto cmd = m_impl->device.request_command_buffer();
    cmd->copy_image_to_buffer(*buffer, *result.image, 0, {}, {width, height, 1}, 0, 0,
                              {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1});
    cmd->barrier(VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                 VK_PIPELINE_STAGE_HOST_BIT, VK_ACCESS_HOST_READ_BIT);
    Vulkan::Fence fence;
    m_impl->device.submit(cmd, &fence);
    fence->wait();

    const auto *mapped = static_cast<const uint8_t *>(
        m_impl->device.map_host_buffer(*buffer, Vulkan::MEMORY_ACCESS_READ_BIT));
    pixels.assign(mapped, mapped + bytes);
    // The scanout keeps PS2 alpha (0x80 = 1.0); the host blit must be opaque.
    for (size_t i = 3; i < pixels.size(); i += 4)
        pixels[i] = 0xFFu;
    m_impl->device.unmap_host_buffer(*buffer, Vulkan::MEMORY_ACCESS_READ_BIT);
    return true;
}

#endif // PS2X_ENABLE_PARALLEL_GS
