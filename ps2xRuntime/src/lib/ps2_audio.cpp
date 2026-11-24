#include "runtime/ps2_audio.h"
#include "runtime/ps2_memory.h"
#include "ps2_host_backend.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace
{
    std::vector<uint8_t> buildWavFromPcm(const int16_t *pcm, size_t sampleCount, uint32_t sampleRate)
    {
        const uint32_t dataSize = static_cast<uint32_t>(sampleCount * 2);
        const uint32_t fileSize = 36 + dataSize;
        std::vector<uint8_t> wav(8 + fileSize);

        uint8_t *p = wav.data();
        p[0] = 'R';
        p[1] = 'I';
        p[2] = 'F';
        p[3] = 'F';
        p[4] = static_cast<uint8_t>(fileSize);
        p[5] = static_cast<uint8_t>(fileSize >> 8);
        p[6] = static_cast<uint8_t>(fileSize >> 16);
        p[7] = static_cast<uint8_t>(fileSize >> 24);
        p[8] = 'W';
        p[9] = 'A';
        p[10] = 'V';
        p[11] = 'E';
        p[12] = 'f';
        p[13] = 'm';
        p[14] = 't';
        p[15] = ' ';
        p[16] = 16;
        p[17] = 0;
        p[18] = 0;
        p[19] = 0;
        p[20] = 1;
        p[21] = 0;
        p[22] = 1;
        p[23] = 0;
        p[24] = static_cast<uint8_t>(sampleRate);
        p[25] = static_cast<uint8_t>(sampleRate >> 8);
        p[26] = static_cast<uint8_t>(sampleRate >> 16);
        p[27] = static_cast<uint8_t>(sampleRate >> 24);
        const uint32_t byteRate = sampleRate * 2;
        p[28] = static_cast<uint8_t>(byteRate);
        p[29] = static_cast<uint8_t>(byteRate >> 8);
        p[30] = static_cast<uint8_t>(byteRate >> 16);
        p[31] = static_cast<uint8_t>(byteRate >> 24);
        p[32] = 2;
        p[33] = 0;
        p[34] = 16;
        p[35] = 0;
        p[36] = 'd';
        p[37] = 'a';
        p[38] = 't';
        p[39] = 'a';
        p[40] = static_cast<uint8_t>(dataSize);
        p[41] = static_cast<uint8_t>(dataSize >> 8);
        p[42] = static_cast<uint8_t>(dataSize >> 16);
        p[43] = static_cast<uint8_t>(dataSize >> 24);
        std::memcpy(p + 44, pcm, dataSize);
        return wav;
    }
}

namespace ps2_vag
{
    bool decode(const uint8_t *data, uint32_t sizeBytes,
                std::vector<int16_t> &outPcm, uint32_t &outSampleRate);
}

struct PS2AudioBackend::Impl
{
    struct TrackedSound
    {
        Sound snd;
        uint32_t sampleKey;
    };
    std::vector<TrackedSound> activeSounds;
};

PS2AudioBackend::PS2AudioBackend() : m_impl(std::make_unique<Impl>())
{
}

PS2AudioBackend::~PS2AudioBackend()
{
    if (m_impl)
        stopAll();
}

void PS2AudioBackend::onVagTransfer(const uint8_t *rdram, uint32_t srcAddr, uint32_t sizeBytes)
{
    if (!rdram || sizeBytes < 48)
        return;

    const uint32_t physAddr = srcAddr & PS2_RAM_MASK;
    if (physAddr + sizeBytes > PS2_RAM_SIZE)
        return;

    std::vector<int16_t> pcm;
    uint32_t sampleRate = 44100;
    if (!ps2_vag::decode(rdram + physAddr, sizeBytes, pcm, sampleRate))
        return;

    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample sample;
    sample.pcm = std::move(pcm);
    sample.sampleRate = sampleRate;
    m_sampleBank[physAddr] = std::move(sample);
    m_mostRecentSampleKey = physAddr;
}

void PS2AudioBackend::onVagTransferFromBuffer(const uint8_t *data, uint32_t sizeBytes, uint32_t keyAddr)
{
    if (!data || sizeBytes < 48)
        return;

    std::vector<int16_t> pcm;
    uint32_t sampleRate = 44100;
    if (!ps2_vag::decode(data, sizeBytes, pcm, sampleRate))
        return;

    const uint32_t physAddr = keyAddr & PS2_RAM_MASK;
    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample sample;
    sample.pcm = std::move(pcm);
    sample.sampleRate = sampleRate;
    m_sampleBank[physAddr] = sample;
    m_mostRecentSampleKey = physAddr;
    m_loadOrderSamples.push_back(std::move(sample));
    m_loadOrderSampleKeys.push_back(physAddr);
    constexpr size_t kMaxLoadOrderSamples = 32;
    if (m_loadOrderSamples.size() > kMaxLoadOrderSamples)
    {
        m_loadOrderSamples.erase(m_loadOrderSamples.begin());
        m_loadOrderSampleKeys.erase(m_loadOrderSampleKeys.begin());
    }
}

namespace
{
    constexpr uint32_t LIBSD_CMD_SET_VOICE = 0x8010u;
}

void PS2AudioBackend::onSoundCommand(uint32_t sid, uint32_t rpcNum,
                                     const uint8_t *sendBuf, uint32_t sendSize,
                                     uint8_t *recvBuf, uint32_t recvSize)
{
    if (sid != 0x80000701u)
        return;

    if ((rpcNum == LIBSD_CMD_SET_VOICE || (rpcNum & 0xFF00u) == 0x8100u) &&
        sendBuf && sendSize >= 20)
    {
        uint32_t sampleAddr = 0;
        uint32_t voiceIndex = 0xFFFFFFFFu;
        for (int vo = 4; vo >= 0 && voiceIndex == 0xFFFFFFFFu; vo -= 4)
        {
            if (vo < static_cast<int>(sendSize))
            {
                uint32_t v = 0;
                std::memcpy(&v, sendBuf + vo, sizeof(v));
                if (v < 24u)
                    voiceIndex = v;
            }
        }

        constexpr uint32_t kMinPlausibleAddr = 0x1000u;
        for (int off = 12; off <= 24 && sampleAddr == 0; off += 4)
        {
            if (sendSize >= static_cast<uint32_t>(off + 4))
            {
                uint32_t cand = 0;
                std::memcpy(&cand, sendBuf + off, sizeof(cand));
                if (cand >= kMinPlausibleAddr && (cand <= PS2_RAM_MASK || (cand & ~PS2_RAM_MASK) == 0))
                    sampleAddr = cand;
            }
        }
        if (sampleAddr == 0)
            sampleAddr = m_mostRecentSampleKey;

        float pitch = 1.0f;
        if (sendSize >= 12)
        {
            uint16_t pitchHalf = 0;
            std::memcpy(&pitchHalf, sendBuf + 8, sizeof(pitchHalf));
            if (pitchHalf != 0)
                pitch = 4096.0f / static_cast<float>(pitchHalf);
        }
        play(sampleAddr, pitch, 1.0f, voiceIndex);
    }
}

void PS2AudioBackend::play(uint32_t sampleAddr, float pitch, float volume, uint32_t voiceIndex)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample *sampleToPlay = nullptr;
    uint32_t sampleKey = 0;

    auto it = m_sampleBank.find(sampleAddr & PS2_RAM_MASK);
    if (it != m_sampleBank.end())
    {
        sampleToPlay = &it->second;
        sampleKey = it->first;
    }
    else if (voiceIndex != 0xFFFFFFFFu &&
             voiceIndex < m_loadOrderSamples.size() &&
             voiceIndex < m_loadOrderSampleKeys.size())
    {
        sampleToPlay = &m_loadOrderSamples[voiceIndex];
        sampleKey = m_loadOrderSampleKeys[voiceIndex];
    }
    else
    {
        it = m_sampleBank.find(m_mostRecentSampleKey);
        if (it == m_sampleBank.end())
            return;
        sampleToPlay = &it->second;
        sampleKey = it->first;
    }
    if (!sampleToPlay || sampleToPlay->pcm.empty())
        return;

    const bool isBgm = (sampleToPlay->pcm.size() > static_cast<size_t>(sampleToPlay->sampleRate * 5));
    playDecodedSample(sampleKey, *sampleToPlay, pitch, volume, isBgm);
}

void PS2AudioBackend::pruneFinishedSounds()
{
#if defined(PLATFORM_VITA)
    return;
#else
    auto &sounds = m_impl->activeSounds;
    auto it = sounds.begin();
    while (it != sounds.end())
    {
        if (!IsSoundPlaying(it->snd))
        {
            UnloadSound(it->snd);
            it = sounds.erase(it);
        }
        else
        {
            ++it;
        }
    }
#endif
}

void PS2AudioBackend::playDecodedSample(uint32_t sampleKey, DecodedSample &sample, float pitch, float volume,
                                        bool isBgm)
{
#if defined(PLATFORM_VITA)
    (void)sampleKey;
    (void)sample;
    (void)pitch;
    (void)volume;
    (void)isBgm;
    return;
#else
    if (!m_audioReady || sample.pcm.empty())
        return;

    pruneFinishedSounds();

    for (const auto &t : m_impl->activeSounds)
    {
        if (t.sampleKey == sampleKey && IsSoundPlaying(t.snd))
            return;
    }

    auto &sounds = m_impl->activeSounds;
    if (isBgm)
    {
        for (auto it = sounds.begin(); it != sounds.end();)
        {
            if (IsSoundPlaying(it->snd))
            {
                StopSound(it->snd);
                UnloadSound(it->snd);
                it = sounds.erase(it);
            }
            else
                ++it;
        }
    }

    constexpr int kMaxConcurrentSounds = 4;
    while (static_cast<int>(sounds.size()) >= kMaxConcurrentSounds)
    {
        StopSound(sounds.front().snd);
        UnloadSound(sounds.front().snd);
        sounds.erase(sounds.begin());
    }

    std::vector<uint8_t> wav = buildWavFromPcm(sample.pcm.data(), sample.pcm.size(), sample.sampleRate);
    Wave wave = LoadWaveFromMemory(".wav", wav.data(), static_cast<int>(wav.size()));
    if (wave.frameCount <= 0)
        return;
    Sound snd = LoadSoundFromWave(wave);
    UnloadWave(wave);
    SetSoundPitch(snd, pitch);
    SetSoundVolume(snd, volume);
    m_impl->activeSounds.push_back({snd, sampleKey});
    PlaySound(snd);
#endif
}

void PS2AudioBackend::stop(uint32_t voiceId)
{
    (void)voiceId;
}

void PS2AudioBackend::stopAll()
{
    std::lock_guard<std::mutex> lock(m_mutex);
#if defined(PLATFORM_VITA)
    return;
#else
    for (auto &t : m_impl->activeSounds)
    {
        StopSound(t.snd);
        UnloadSound(t.snd);
    }
    m_impl->activeSounds.clear();
#endif
}

namespace
{
    constexpr size_t kRingFrames = 48000u;   // 1 s capacity
    constexpr size_t kTargetFrames = 2048u;  // ~43 ms of latency
    constexpr size_t kMaxFrames = 9600u;     // trim beyond 200 ms
    PS2AudioBackend *g_streamOwner = nullptr;
#if !defined(PLATFORM_VITA)
    AudioStream g_stream{};

    void streamCallback(void *buffer, unsigned int frames)
    {
        if (g_streamOwner)
            g_streamOwner->pullPcm(static_cast<int16_t *>(buffer), frames);
        else
            std::memset(buffer, 0, static_cast<size_t>(frames) * 4u);
    }
#endif
}

void PS2AudioBackend::startStream()
{
#if !defined(PLATFORM_VITA)
    if (!m_audioReady || m_streamActive)
        return;
    if (std::getenv("PS2X_AUDIO_MUTE"))
        return; // test runs: no sound on the speakers (PS2X_AUDIO_DUMP still records)
    {
        std::lock_guard<std::mutex> lock(m_streamMutex);
        m_ring.assign(kRingFrames * 2u, 0);
        m_ringRead = m_ringCount = 0;
    }
    SetAudioStreamBufferSizeDefault(1024);
    g_stream = LoadAudioStream(48000, 16, 2);
    if (!IsAudioStreamValid(g_stream))
        return;
    g_streamOwner = this;
    SetAudioStreamCallback(g_stream, streamCallback);
    PlayAudioStream(g_stream);
    m_streamActive = true;
#endif
}

void PS2AudioBackend::stopStream()
{
#if !defined(PLATFORM_VITA)
    if (!m_streamActive)
        return;
    StopAudioStream(g_stream);
    UnloadAudioStream(g_stream);
    g_streamOwner = nullptr;
    m_streamActive = false;
#endif
}

void PS2AudioBackend::pushPcm(const int16_t *frames, size_t count)
{
    if (!frames || count == 0)
        return;
    {
        // PS2X_AUDIO_DUMP=<file>: raw interleaved s16le 48 kHz stereo of the SPU2 output.
        static FILE *dump = []() -> FILE *
        {
            const char *path = std::getenv("PS2X_AUDIO_DUMP");
            return path ? std::fopen(path, "wb") : nullptr;
        }();
        if (dump)
        {
            std::fwrite(frames, 4u, count, dump);
            std::fflush(dump);
        }
    }
    if (!m_streamActive)
        return;
    std::vector<int16_t> mixed;
    if (m_musicActive.load())
    {
        std::lock_guard<std::mutex> musicLock(m_musicMutex);
        if (!m_music.empty())
        {
            mixed.assign(frames, frames + count * 2u);
            for (size_t i = 0; i < count * 2u; ++i)
            {
                mixed[i] = static_cast<int16_t>(std::clamp<int32_t>(mixed[i] + m_music[m_musicPos], -32768, 32767));
                if (++m_musicPos >= m_music.size())
                    m_musicPos = 0;
            }
            frames = mixed.data();
        }
    }
    std::lock_guard<std::mutex> lock(m_streamMutex);
    const size_t cap = m_ring.size() / 2u;
    for (size_t i = 0; i < count; ++i)
    {
        const size_t w = (m_ringRead + m_ringCount) % cap;
        m_ring[w * 2u] = frames[i * 2u];
        m_ring[w * 2u + 1u] = frames[i * 2u + 1u];
        if (m_ringCount < cap)
            ++m_ringCount;
        else
            m_ringRead = (m_ringRead + 1u) % cap;
    }
    m_pushed += count;
    if (m_ringCount > kMaxFrames)
    {
        // Emulation ran ahead of the sound card: drop the oldest audio.
        const size_t drop = m_ringCount - kTargetFrames;
        m_ringRead = (m_ringRead + drop) % cap;
        m_ringCount -= drop;
        m_dropped += drop;
    }
    static const bool trace = std::getenv("PS2X_SPU2_TRACE") != nullptr;
    if (trace && (m_pushed / 48000u) != ((m_pushed - count) / 48000u) && (m_pushed / 48000u) % 5u == 0u)
        std::fprintf(stderr, "[audio] pushed=%llus buffered=%zu underruns=%llu dropped=%llu\n",
                     static_cast<unsigned long long>(m_pushed / 48000u), m_ringCount,
                     static_cast<unsigned long long>(m_underruns), static_cast<unsigned long long>(m_dropped));
}

void PS2AudioBackend::pullPcm(int16_t *out, size_t count)
{
    std::lock_guard<std::mutex> lock(m_streamMutex);
    const size_t cap = m_ring.size() / 2u;
    size_t i = 0;
    for (; i < count && m_ringCount > 0 && cap > 0; ++i)
    {
        out[i * 2u] = m_ring[m_ringRead * 2u];
        out[i * 2u + 1u] = m_ring[m_ringRead * 2u + 1u];
        m_ringRead = (m_ringRead + 1u) % cap;
        --m_ringCount;
    }
    if (i < count)
    {
        ++m_underruns;
        std::memset(out + i * 2u, 0, (count - i) * 4u);
    }
}

namespace
{
    // Minimal RIFF/WAVE PCM reader -> interleaved stereo int16 at 48 kHz.
    bool loadWav48k(const std::string &path, std::vector<int16_t> &out)
    {
        FILE *f = std::fopen(path.c_str(), "rb");
        if (!f)
            return false;
        std::vector<uint8_t> data;
        std::fseek(f, 0, SEEK_END);
        const long size = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        if (size > 12)
        {
            data.resize(static_cast<size_t>(size));
            if (std::fread(data.data(), 1, data.size(), f) != data.size())
                data.clear();
        }
        std::fclose(f);
        if (data.size() < 12 || std::memcmp(data.data(), "RIFF", 4) != 0 || std::memcmp(data.data() + 8, "WAVE", 4) != 0)
            return false;
        uint16_t format = 0, channels = 0, bits = 0;
        uint32_t rate = 0;
        const uint8_t *pcm = nullptr;
        size_t pcmBytes = 0;
        for (size_t pos = 12; pos + 8 <= data.size();)
        {
            uint32_t len = 0;
            std::memcpy(&len, data.data() + pos + 4, 4);
            const uint8_t *body = data.data() + pos + 8;
            const size_t avail = std::min<size_t>(len, data.size() - pos - 8);
            if (std::memcmp(data.data() + pos, "fmt ", 4) == 0 && avail >= 16)
            {
                std::memcpy(&format, body, 2);
                std::memcpy(&channels, body + 2, 2);
                std::memcpy(&rate, body + 4, 4);
                std::memcpy(&bits, body + 14, 2);
            }
            else if (std::memcmp(data.data() + pos, "data", 4) == 0)
            {
                pcm = body;
                pcmBytes = avail;
            }
            pos += 8 + len + (len & 1u);
        }
        if ((format != 1 && format != 0xFFFE) || !pcm || channels == 0 || channels > 2 || rate == 0 || (bits != 16 && bits != 8))
            return false;
        const size_t bytesPerFrame = static_cast<size_t>(channels) * (bits / 8u);
        const size_t frames = pcmBytes / bytesPerFrame;
        auto sampleAt = [&](size_t frame, int ch) -> int32_t
        {
            const uint8_t *p = pcm + frame * bytesPerFrame + static_cast<size_t>(std::min<int>(ch, channels - 1)) * (bits / 8u);
            if (bits == 8)
                return (static_cast<int32_t>(*p) - 128) << 8;
            int16_t v;
            std::memcpy(&v, p, 2);
            return v;
        };
        const double step = static_cast<double>(rate) / 48000.0;
        const size_t outFrames = static_cast<size_t>(static_cast<double>(frames) / step);
        out.resize(outFrames * 2u);
        for (size_t i = 0; i < outFrames; ++i)
        {
            const double srcPos = static_cast<double>(i) * step;
            const size_t a = static_cast<size_t>(srcPos);
            const size_t b = std::min(a + 1u, frames - 1u);
            const double t = srcPos - static_cast<double>(a);
            for (int ch = 0; ch < 2; ++ch)
                out[i * 2u + static_cast<size_t>(ch)] = static_cast<int16_t>(sampleAt(a, ch) + (sampleAt(b, ch) - sampleAt(a, ch)) * t);
        }
        return !out.empty();
    }
}

bool PS2AudioBackend::setMusicOverride(const std::string &path)
{
    if (path == m_musicPath)
        return m_musicActive.load() || path.empty();
    std::vector<int16_t> pcm;
    const bool ok = path.empty() || loadWav48k(path, pcm);
    {
        std::lock_guard<std::mutex> lock(m_musicMutex);
        m_music = std::move(pcm);
        m_musicPos = 0;
        m_musicPath = path;
        m_musicActive.store(ok && !m_music.empty());
    }
    if (!ok)
        std::fprintf(stderr, "[audio] cannot load music override: %s\n", path.c_str());
    return ok;
}
