#ifndef PS2_DEBUG_PANEL_H
#define PS2_DEBUG_PANEL_H

// Games can relocate the debugger toggle (raylib KeyboardKey value) and start
// with the panel hidden, e.g. -DPS2X_DEBUG_UI_TOGGLE_KEY=297 (KEY_F8).
#ifndef PS2X_DEBUG_UI_TOGGLE_KEY
#define PS2X_DEBUG_UI_TOGGLE_KEY 290 /* KEY_F1 */
#endif
#ifndef PS2X_DEBUG_UI_START_VISIBLE
#define PS2X_DEBUG_UI_START_VISIBLE 1
#endif

class PS2Runtime;

class PS2DebugPanel
{
public:
    void initialize();
    void shutdown();
    void draw(PS2Runtime &runtime);

    bool isVisible() const { return m_visible; }
    void setVisible(bool visible) { m_visible = visible; }
    void toggleVisible() { m_visible = !m_visible; }

private:
    bool m_initialized = false;
    bool m_visible = PS2X_DEBUG_UI_START_VISIBLE != 0;
    bool m_showRegisters = true;
    unsigned int m_memoryAddress = 0x00100000u;
    unsigned int m_memoryBytes = 0x100u;
};

#endif // PS2_DEBUG_PANEL_H
