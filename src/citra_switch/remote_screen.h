// Remote screen: streams the 3DS screens to a phone browser over WebSocket and feeds the
// phone's touches back into the emulated touchscreen.
//
// Two modes, chosen by the phone:
//   Bottom - only the bottom screen goes to the phone; the Switch keeps showing the top.
//   Full   - both screens go to the phone, which becomes the display. The phone lays them
//            out itself (stacked when held upright, side by side when held sideways).
#pragma once

#include <cstdint>
#include <string>

namespace VideoCore {
class RendererBase;
}

namespace RemoteScreen {

enum class Mode : std::uint8_t {
    Bottom = 0,
    Full = 1,
};

struct Touch {
    bool pressed{};
    float x{}; // 0..1 across the 320px bottom screen
    float y{}; // 0..1 down the 240px bottom screen
};

// Starts the HTTP + WebSocket server thread. web_root must contain index.html.
bool Start(std::uint16_t port, const std::string& web_root);
void Stop();

bool IsClientConnected();
Mode CurrentMode();

// Emulation thread, once per loop iteration. Requests a capture at most kCaptureFps times a
// second, and only while a phone is connected.
void MaybeRequestCapture(VideoCore::RendererBase& renderer);

// Input thread: current phone touch. Returns false when no phone is connected.
bool GetTouch(Touch& out);

} // namespace RemoteScreen
