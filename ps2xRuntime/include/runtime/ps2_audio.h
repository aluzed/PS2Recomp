#ifndef PS2_AUDIO_H
#define PS2_AUDIO_H

#include <atomic>
#include <cstdint>
#include <string>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class PS2AudioBackend
{
public:
    PS2AudioBackend();
    ~PS2AudioBackend();

    void onVagTransfer(const uint8_t *rdram, uint32_t srcAddr, uint32_t sizeBytes);
    void onVagTransferFromBuffer(const uint8_t *data, uint32_t sizeBytes, uint32_t keyAddr);
    void onSoundCommand(uint32_t sid, uint32_t rpcNum,
                        const uint8_t *sendBuf, uint32_t sendSize,
                        uint8_t *recvBuf, uint32_t recvSize);

    void play(uint32_t sampleAddr, float pitch = 1.0f, float volume = 1.0f,
              uint32_t voiceIndex = 0xFFFFFFFFu);
    void stop(uint32_t voiceId);
    void stopAll();
    void setAudioReady(bool ready) { m_audioReady = ready; }

    // SPU2 output (IOP emulator): interleaved stereo 16-bit frames at 48 kHz,
    // played through a host audio stream. Audio never paces the game: the
    // buffer is trimmed when it runs ahead and padded with silence when dry.
    void startStream();
    void stopStream();
    void pushPcm(const int16_t *frames, size_t count);
    void pullPcm(int16_t *out, size_t count);

    // Host music track mixed over the SPU2 output (custom stage music).
    // PCM WAV (8/16-bit, mono/stereo, any rate; resampled to 48 kHz), looped.
    // Same path again keeps playing; an empty path stops. Returns false when
    // the file cannot be read.
    bool setMusicOverride(const std::string &path);
    [[nodiscard]] bool musicOverrideActive() const { return m_musicActive.load(); }

private:
    struct DecodedSample
    {
        std::vector<int16_t> pcm;
        uint32_t sampleRate = 44100;
    };

    struct Impl;
    std::unique_ptr<Impl> m_impl;
    bool m_audioReady = false;
    uint32_t m_mostRecentSampleKey = 0;
    std::vector<DecodedSample> m_loadOrderSamples;
    std::vector<uint32_t> m_loadOrderSampleKeys;
    std::unordered_map<uint32_t, DecodedSample> m_sampleBank;
    std::mutex m_mutex;
    std::mutex m_streamMutex;
    std::vector<int16_t> m_ring;      // interleaved stereo
    size_t m_ringRead = 0, m_ringCount = 0;
    bool m_streamActive = false;
    uint64_t m_underruns = 0, m_dropped = 0, m_pushed = 0;
    std::mutex m_musicMutex;
    std::vector<int16_t> m_music;     // interleaved stereo, 48 kHz
    size_t m_musicPos = 0;
    std::string m_musicPath;
    std::atomic<bool> m_musicActive{false};

    void playDecodedSample(uint32_t sampleKey, DecodedSample &sample, float pitch, float volume,
                          bool isBgm = false);
    void pruneFinishedSounds();
};

#endif
