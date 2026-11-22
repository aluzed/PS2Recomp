#ifndef PS2_PAD_H
#define PS2_PAD_H

#include <cstddef>
#include <cstdint>

class PSPadBackend
{
public:
    PSPadBackend() = default;
    ~PSPadBackend() = default;

    bool readState(int port, int slot, uint8_t *data, size_t size);

    // Called after the host state has been written into data (32-byte pad
    // reply: data[2..3] = buttons, active low; data[4..7] = sticks). Lets a
    // game layer inject or replace input (scripted input, replays, netplay).
    using ReadHook = void (*)(int port, int slot, uint8_t *data, void *user);
    void setReadHook(ReadHook hook, void *user)
    {
        m_readHook = hook;
        m_readHookUser = user;
    }

private:
    ReadHook m_readHook = nullptr;
    void *m_readHookUser = nullptr;
};

#endif
