#include "citra_switch/remote_screen.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csetjmp>
#include <cstdio>
#include <cstdlib>

#include <cryptopp/sha.h>
#include <jpeglib.h>

#include "common/horizon_thread.h"
#include "common/logging/log.h"
#include "common/math_util.h"
#include "common/settings.h"
#include "core/frontend/framebuffer_layout.h"
#include "video_core/renderer_base.h"

namespace RemoteScreen {
namespace {

constexpr int kCaptureFps = 30;
constexpr int kJpegQuality = 80;
constexpr std::size_t kMaxClientMessage = 1024;
constexpr char kWsGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// Message types (first byte of every binary WebSocket message)
// server -> phone: [1][mode u8][w u16][h u16][top x,y,w,h u16][bottom x,y,w,h u16][JPEG]
constexpr std::uint8_t kMsgFrame = 1;
// phone -> server: [2][pressed u8][x u16][y u16], x/y 0..65535 on the bottom screen
constexpr std::uint8_t kMsgTouch = 2;
// phone -> server: [3][mode u8]
constexpr std::uint8_t kMsgMode = 3;

// Where each 3DS screen sits inside the captured image. A zero width means "not sent".
struct Packing {
    std::uint16_t width, height;
    std::uint16_t top[4];    // x, y, w, h
    std::uint16_t bottom[4]; // x, y, w, h
};
// Bottom only: the 320x240 bottom screen.
constexpr Packing kPackBottom{320, 240, {0, 0, 0, 0}, {0, 0, 320, 240}};
// Full: top 400x240 above the bottom 320x240, centred. The phone rearranges them freely.
constexpr Packing kPackFull{400, 480, {0, 0, 400, 240}, {40, 240, 320, 240}};
constexpr std::size_t kMaxPixels = 400 * 480;

const Packing& PackingFor(Mode mode) {
    return mode == Mode::Full ? kPackFull : kPackBottom;
}

std::atomic<bool> s_running{false};
std::atomic<bool> s_connected{false};
std::atomic<Mode> s_mode{Mode::Bottom};
std::thread s_thread;
int s_listen_fd = -1;
std::string s_web_root;

// Latest capture, BGRA, written by the GPU thread and read (then JPEG-encoded) by the server.
struct Frame {
    std::vector<std::uint8_t> bgra;
    Packing packing{};
    Mode mode{};
    bool invert_y{};
};
std::mutex s_frame_mutex;
Frame s_frame;
std::uint32_t s_frame_seq = 0;

// Capture target for RequestScreenshot (BGRA, as the Vulkan renderer writes it).
std::vector<std::uint8_t> s_capture(kMaxPixels * 4);
Mode s_capture_mode{};
std::atomic<bool> s_capture_pending{false};
std::chrono::steady_clock::time_point s_last_capture{};

std::mutex s_touch_mutex;
Touch s_touch{};

// ---------------------------------------------------------------- helpers

std::string Base64(const std::uint8_t* d, std::size_t n) {
    static constexpr char t[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < n; i += 3) {
        std::uint32_t v = d[i] << 16;
        if (i + 1 < n) v |= d[i + 1] << 8;
        if (i + 2 < n) v |= d[i + 2];
        out += t[(v >> 18) & 63];
        out += t[(v >> 12) & 63];
        out += i + 1 < n ? t[(v >> 6) & 63] : '=';
        out += i + 2 < n ? t[v & 63] : '=';
    }
    return out;
}

bool SendAll(int fd, const void* data, std::size_t len) {
    const auto* p = static_cast<const std::uint8_t*>(data);
    while (len > 0) {
        const ssize_t n = send(fd, p, len, 0);
        if (n <= 0) return false;
        p += n;
        len -= static_cast<std::size_t>(n);
    }
    return true;
}

bool ReadHttpRequest(int fd, std::string& req) {
    char buf[1024];
    while (req.find("\r\n\r\n") == std::string::npos) {
        if (req.size() > 8192) return false;
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(fd, &rf);
        timeval tv{2, 0};
        if (select(fd + 1, &rf, nullptr, nullptr, &tv) <= 0) return false;
        const ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) return false;
        req.append(buf, static_cast<std::size_t>(n));
    }
    return true;
}

std::string HeaderValue(const std::string& req, const std::string& lower_name) {
    std::string low = req;
    std::transform(low.begin(), low.end(), low.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    auto pos = low.find("\r\n" + lower_name + ":");
    if (pos == std::string::npos) return {};
    pos += 2 + lower_name.size() + 1;
    const auto end = req.find("\r\n", pos);
    std::string v = req.substr(pos, end - pos);
    v.erase(0, v.find_first_not_of(' '));
    v.erase(v.find_last_not_of(' ') + 1);
    return v;
}

void ServeHttp(int fd, const std::string& req) {
    const bool root = req.rfind("GET / ", 0) == 0 || req.rfind("GET /index.html ", 0) == 0;
    std::string body;
    if (root) {
        std::ifstream f(s_web_root + "/index.html", std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        body = ss.str();
    }
    std::string head;
    if (!body.empty()) {
        head = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
               "Cache-Control: no-store\r\nContent-Length: " +
               std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
    } else {
        body = "not found";
        head = "HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\nConnection: close\r\n\r\n";
    }
    SendAll(fd, head.data(), head.size()) && SendAll(fd, body.data(), body.size());
}

bool Handshake(int fd, const std::string& req) {
    const std::string key = HeaderValue(req, "sec-websocket-key") + kWsGuid;
    std::array<CryptoPP::byte, CryptoPP::SHA1::DIGESTSIZE> digest{};
    CryptoPP::SHA1().CalculateDigest(digest.data(),
                                     reinterpret_cast<const CryptoPP::byte*>(key.data()),
                                     key.size());
    const std::string resp = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                             "Connection: Upgrade\r\nSec-WebSocket-Accept: " +
                             Base64(digest.data(), digest.size()) + "\r\n\r\n";
    return SendAll(fd, resp.data(), resp.size());
}

bool SendWsFrame(int fd, std::uint8_t opcode, const std::uint8_t* p, std::size_t n) {
    std::uint8_t h[10];
    std::size_t hl = 0;
    h[hl++] = 0x80 | opcode;
    if (n < 126) {
        h[hl++] = static_cast<std::uint8_t>(n);
    } else if (n < 65536) {
        h[hl++] = 126;
        h[hl++] = static_cast<std::uint8_t>(n >> 8);
        h[hl++] = static_cast<std::uint8_t>(n);
    } else {
        h[hl++] = 127;
        for (int i = 7; i >= 0; --i) h[hl++] = static_cast<std::uint8_t>(std::uint64_t(n) >> (8 * i));
    }
    return SendAll(fd, h, hl) && (n == 0 || SendAll(fd, p, n));
}

void HandleMessage(const std::vector<std::uint8_t>& m) {
    if (m.size() == 6 && m[0] == kMsgTouch) {
        const std::uint16_t x = m[2] | (m[3] << 8);
        const std::uint16_t y = m[4] | (m[5] << 8);
        std::scoped_lock lock(s_touch_mutex);
        s_touch = {m[1] != 0, x / 65535.f, y / 65535.f};
    } else if (m.size() == 2 && m[0] == kMsgMode && m[1] <= 1) {
        const Mode mode = static_cast<Mode>(m[1]);
        s_mode = mode; // the next capture has a new shape, so it is always sent
    }
}

// Parses every complete client frame in rx. Returns false when the connection should close.
bool ProcessIncoming(int fd, std::vector<std::uint8_t>& rx) {
    for (;;) {
        if (rx.size() < 2) return true;
        const std::uint8_t op = rx[0] & 0x0F;
        const bool masked = rx[1] & 0x80;
        std::size_t len = rx[1] & 0x7F;
        std::size_t pos = 2;
        if (len == 126) {
            if (rx.size() < 4) return true;
            len = (rx[2] << 8) | rx[3];
            pos = 4;
        } else if (len == 127) {
            return false;
        }
        if (!masked || len > kMaxClientMessage) return false;
        if (rx.size() < pos + 4 + len) return true;

        const std::uint8_t* mask = &rx[pos];
        std::vector<std::uint8_t> payload(rx.begin() + pos + 4, rx.begin() + pos + 4 + len);
        for (std::size_t i = 0; i < payload.size(); ++i) payload[i] ^= mask[i & 3];
        rx.erase(rx.begin(), rx.begin() + pos + 4 + len);

        switch (op) {
        case 0x8: // close
            SendWsFrame(fd, 0x8, nullptr, 0);
            return false;
        case 0x9: // ping
            if (!SendWsFrame(fd, 0xA, payload.data(), payload.size())) return false;
            break;
        case 0x2: // binary
            HandleMessage(payload);
            break;
        default:
            break;
        }
    }
}

struct JpegError {
    jpeg_error_mgr mgr;
    std::jmp_buf jump;
};

void OnJpegError(j_common_ptr cinfo) {
    std::longjmp(reinterpret_cast<JpegError*>(cinfo->err)->jump, 1);
}

// Encodes into out after its current contents. Kept in a struct so the output buffer
// survives a longjmp from libjpeg's error handler.
struct JpegEncoder {
    unsigned char* mem = nullptr;
    unsigned long mem_size = 0;

    ~JpegEncoder() {
        std::free(mem);
    }

    bool Encode(const Frame& f, std::vector<std::uint8_t>& out) {
        jpeg_compress_struct cinfo{};
        JpegError err{};
        cinfo.err = jpeg_std_error(&err.mgr);
        err.mgr.error_exit = OnJpegError;
        if (setjmp(err.jump)) {
            jpeg_destroy_compress(&cinfo);
            return false;
        }
        jpeg_create_compress(&cinfo);
        unsigned long size = mem_size;
        jpeg_mem_dest(&cinfo, &mem, &size); // reuses mem, grows it if needed

        const std::uint32_t w = f.packing.width;
        const std::uint32_t h = f.packing.height;
        cinfo.image_width = w;
        cinfo.image_height = h;
        cinfo.input_components = 4;
        cinfo.in_color_space = JCS_EXT_BGRA;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, kJpegQuality, TRUE);
        cinfo.dct_method = JDCT_IFAST;

        jpeg_start_compress(&cinfo, TRUE);
        while (cinfo.next_scanline < h) {
            const std::uint32_t row = f.invert_y ? h - 1 - cinfo.next_scanline : cinfo.next_scanline;
            JSAMPROW ptr = const_cast<std::uint8_t*>(f.bgra.data() + row * w * 4);
            jpeg_write_scanlines(&cinfo, &ptr, 1);
        }
        jpeg_finish_compress(&cinfo);
        jpeg_destroy_compress(&cinfo);

        mem_size = std::max(mem_size, size);
        out.insert(out.end(), mem, mem + size);
        return true;
    }
};

void PutU16(std::vector<std::uint8_t>& v, std::uint16_t x) {
    v.push_back(static_cast<std::uint8_t>(x));
    v.push_back(static_cast<std::uint8_t>(x >> 8));
}

bool SendFrame(int fd, JpegEncoder& encoder, const Frame& f, std::vector<std::uint8_t>& out) {
    out.clear();
    out.push_back(kMsgFrame);
    out.push_back(static_cast<std::uint8_t>(f.mode));
    PutU16(out, f.packing.width);
    PutU16(out, f.packing.height);
    for (const std::uint16_t v : f.packing.top) PutU16(out, v);
    for (const std::uint16_t v : f.packing.bottom) PutU16(out, v);
    if (!encoder.Encode(f, out)) return true; // skip a bad frame, keep the connection
    return SendWsFrame(fd, 0x2, out.data(), out.size());
}

void RunWebSocket(int fd) {
    LOG_INFO(Frontend, "RemoteScreen: phone connected");
    s_connected = true;

    s_mode = Mode::Bottom;
    JpegEncoder encoder;
    std::vector<std::uint8_t> rx, out;
    Frame snapshot;
    std::uint32_t sent_seq = ~0u;

    while (s_running) {
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(fd, &rf);
        timeval tv{0, 4000};
        const int r = select(fd + 1, &rf, nullptr, nullptr, &tv);
        if (r < 0) break;
        if (r > 0 && FD_ISSET(fd, &rf)) {
            std::uint8_t buf[512];
            const ssize_t n = recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) break;
            rx.insert(rx.end(), buf, buf + n);
            if (!ProcessIncoming(fd, rx)) break;
        }

        std::uint32_t seq;
        {
            std::scoped_lock lock(s_frame_mutex);
            seq = s_frame_seq;
            if (seq != sent_seq) snapshot = s_frame;
        }
        if (seq != sent_seq) {
            sent_seq = seq;
            if (snapshot.mode != s_mode) continue; // stale capture from before a mode switch
            if (!SendFrame(fd, encoder, snapshot, out)) break;
        }
    }

    s_connected = false;
    {
        std::scoped_lock lock(s_touch_mutex);
        s_touch = {};
    }
    close(fd);
    LOG_INFO(Frontend, "RemoteScreen: phone disconnected");
}

void ServerLoop() {
    // Keep socket work and JPEG encoding off the emulation core.
    Common::Horizon::PinCurrentThread(Common::Horizon::CoreAudio);
    while (s_running) {
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(s_listen_fd, &rf);
        timeval tv{0, 200000};
        if (select(s_listen_fd + 1, &rf, nullptr, nullptr, &tv) <= 0) continue;

        const int fd = accept(s_listen_fd, nullptr, nullptr);
        if (fd < 0) continue;
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        std::string req;
        if (!ReadHttpRequest(fd, req)) {
            close(fd);
            continue;
        }
        // One phone at a time: the WebSocket session runs on this thread until it ends.
        if (!HeaderValue(req, "sec-websocket-key").empty()) {
            if (Handshake(fd, req)) {
                RunWebSocket(fd);
            } else {
                close(fd);
            }
        } else {
            ServeHttp(fd, req);
            close(fd);
        }
    }
}

// GPU thread: publishes the capture if it changed. Encoding happens on the server thread.
void OnCaptureComplete(bool invert_y) {
    const Packing& p = PackingFor(s_capture_mode);
    const std::size_t bytes = std::size_t(p.width) * p.height * 4;
    {
        std::scoped_lock lock(s_frame_mutex);
        const bool same_shape = s_frame.mode == s_capture_mode && s_frame.bgra.size() == bytes;
        if (!same_shape || std::memcmp(s_frame.bgra.data(), s_capture.data(), bytes) != 0) {
            s_frame.bgra.assign(s_capture.begin(), s_capture.begin() + bytes);
            s_frame.packing = p;
            s_frame.mode = s_capture_mode;
            s_frame.invert_y = invert_y;
            ++s_frame_seq;
        }
    }
    s_capture_pending = false;
}

Layout::FramebufferLayout MakeLayout(const Packing& p) {
    const auto rect = [](const std::uint16_t r[4]) {
        return Common::Rectangle<std::uint32_t>{r[0], r[1], std::uint32_t(r[0] + r[2]),
                                               std::uint32_t(r[1] + r[3])};
    };
    Layout::FramebufferLayout layout{};
    layout.width = p.width;
    layout.height = p.height;
    layout.top_screen_enabled = p.top[2] != 0;
    layout.bottom_screen_enabled = p.bottom[2] != 0;
    // A disabled screen still needs a sane rectangle for the draw code.
    layout.top_screen = rect(layout.top_screen_enabled ? p.top : p.bottom);
    layout.bottom_screen = rect(p.bottom);
    layout.is_rotated = true;
    layout.render_3d_mode = Settings::StereoRenderOption::Off; // one eye is enough for a phone
    return layout;
}

} // namespace

bool Start(std::uint16_t port, const std::string& web_root) {
    if (s_running) return true;
    s_web_root = web_root;

    s_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s_listen_fd < 0) return false;
    int one = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(s_listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 ||
        listen(s_listen_fd, 2) < 0) {
        close(s_listen_fd);
        s_listen_fd = -1;
        return false;
    }

    s_running = true;
    s_thread = std::thread(ServerLoop);
    LOG_INFO(Frontend, "RemoteScreen: listening on port {}", port);
    return true;
}

void Stop() {
    if (!s_running) return;
    s_running = false;
    if (s_thread.joinable()) s_thread.join();
    close(s_listen_fd);
    s_listen_fd = -1;
}

bool IsClientConnected() {
    return s_connected;
}

void MaybeRequestCapture(VideoCore::RendererBase& renderer) {
    if (!s_connected) return;
    const auto now = std::chrono::steady_clock::now();
    // RequestScreenshot silently drops a request while another screenshot is in flight,
    // so a capture that never completed is abandoned after a second.
    if (s_capture_pending && now - s_last_capture < std::chrono::seconds(1)) return;
    if (now - s_last_capture < std::chrono::milliseconds(1000 / kCaptureFps)) return;
    s_last_capture = now;

    s_capture_mode = s_mode;
    s_capture_pending = true;
    renderer.RequestScreenshot(s_capture.data(), OnCaptureComplete,
                               MakeLayout(PackingFor(s_capture_mode)));
}

Mode CurrentMode() {
    return s_mode;
}

bool GetTouch(Touch& out) {
    if (!s_connected) return false;
    std::scoped_lock lock(s_touch_mutex);
    out = s_touch;
    return true;
}

} // namespace RemoteScreen
