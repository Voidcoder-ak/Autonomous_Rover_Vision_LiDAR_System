// lidar_stop.cpp
// ---------------------------------------------------------------------------
// LiDAR step 1: "Is something in front of the rover, closer than the stop
// distance?"  -> STOP.  "Has it been clear for a moment?" -> CLEAR.
// Also measures free space on the left and right (the input the bypass
// feature will need later).
//
// Pipeline per scan (~7 times a second on the X2):
//   1. Get one full 360-degree scan: ~430 points of (angle, distance).
//   2. Throw away invalid points (0 m, closer than 0.10 m, farther than 8 m).
//   3. Rotate the angles so 0 deg = the rover's forward direction.
//   4. Convert each point from (angle, distance) to (x ahead, y left) in metres.
//   5. Ignore points that hit the rover itself.
//   6. Look inside a RECTANGLE as wide as the rover (+ margin) ahead of it.
//   7. Nearest obstacle = the 2nd-closest point in that rectangle (not the 1st,
//      so a single dust/noise point can't stop the rover).   <-- CHANGED 30-Sep (1/8): said "3rd", fixed to "2nd" to match the code
//   8. Stop/clear decision with hysteresis (stop at 30 cm, resume at 40 cm).
//   9. (optional, --serial) Tell the ESP32 every 100 ms: "S:0" = stop, "S:100" = go.   <-- CHANGED 30-Sep (1/8): new step
//
// Build:  mkdir build && cd build && cmake .. && make
// Run:    ./lidar_stop --port /dev/ttyUSB0              (radar window)
//         ./lidar_stop --port /dev/ttyUSB0 --calib      (find which way is "forward")
//         ./lidar_stop --sim                             (no LiDAR needed: fake room + box)
//         ./lidar_stop --port /dev/ttyUSB0 --offset 90 --headless
//         ./lidar_stop --port /dev/ttyUSB0 --serial /dev/ttyUSB1   (also drive the ESP32)   <-- CHANGED 30-Sep (1/8)
// ---------------------------------------------------------------------------

#include "CYdLidar.h"
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>   // CHANGED 30-Sep (2/8): new, for the heartbeat thread's on/off flag
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>    // CHANGED 30-Sep (2/8): new, lock shared between main loop and heartbeat thread
#include <random>
#include <string>
#include <thread>
#include <vector>

// >>> CHANGED 30-Sep (2/8): new, Linux serial port headers (open, termios settings, write/close)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
// <<< END CHANGE

using namespace ydlidar;
using Clock = std::chrono::steady_clock;  // CHANGED 30-Sep (2/8): new, short name for the timer clock

static constexpr float kPi = 3.14159265f;
static float deg2rad(float d) { return d * kPi / 180.0f; }
static float rad2deg(float r) { return r * 180.0f / kPi; }

// ===========================================================================
// 1. Settings
// ===========================================================================
struct Config {
    // --- LiDAR (X2 values from the YDLidar SDK dataset table; don't change) ---
    std::string port = "/dev/ttyUSB0";
    int baud = 115200;         // X2 = 115200
    int sampleRateK = 3;       // X2 = 3 kHz
    bool singleChannel = true; // X2 only talks, never listens
    float minRangeM = 0.10f, maxRangeM = 8.0f;  // X2 spec

    // --- Mounting (MEASURE THESE on your rover) ---
    float forwardOffsetDeg = 0.0f;  // added to every angle so 0 deg = rover forward (--offset)
    bool mirror = false;            // flip left/right if the view is mirrored (--mirror)
    float lidarToFrontM = 0.08f;    // LiDAR centre -> front bumper
    float lidarToRearM = 0.15f;     // LiDAR centre -> rear edge
    float halfWidthM = 0.10f;       // half the rover's width
    float marginM = 0.05f;          // extra clearance on each side of the corridor

    // --- Decision ---
    float stopDistM = 0.30f;    // STOP if an obstacle is closer than this (measured from the FRONT bumper)
    float resumeDistM = 0.40f;  // CLEAR only once it's farther than this (hysteresis gap = 10 cm)
    float neighbourTolM = 0.05f;  // lonely-point filter: a neighbour must be within 5 cm...
    float neighbourMaxDeg = 3.0f; // ...and within 3 degrees
    int kthNearest = 2;         // use the 2nd-nearest point in the corridor (extra safety vs a stray point;
                                // 2 not 3, so a thin chair leg that returns only 2 points still counts)
    int scansToStop = 1;        // stop on the FIRST scan that says so (safety first)
    int scansToResume = 5;      // resume only after 5 clear scans in a row (~0.7 s)

    // --- Side clearance (for the future bypass) ---
    float sideMinDeg = 30.0f, sideMaxDeg = 100.0f;  // sectors on each side

    // >>> CHANGED 30-Sep (3/8): new settings for the ESP32 link
    // --- ESP32 link (only used with --serial) ---
    int espBaud = 115200;       // must match Serial.begin(...) on the ESP32
    int sendEveryMs = 100;      // heartbeat: ESP32 failsafe is 200 ms, so send twice as often
    int staleScanMs = 400;      // no new scan for this long = LiDAR problem -> send STOP
    // <<< END CHANGE
};

// One point in the rover's frame: x = metres ahead of the LiDAR, y = metres to the left.
struct Pt {
    float x, y, range, angleDeg;
};

// ===========================================================================
// 2. Where scans come from: the real X2, or a simulator for testing at a desk
// ===========================================================================
class ScanSource {
public:
    virtual ~ScanSource() = default;
    virtual bool start() = 0;
    // Fills (angleRad, rangeM) pairs for one full rotation. Angle is counter-clockwise, left positive.
    virtual bool next(std::vector<std::pair<float, float>>& out, float& scanHz) = 0;
    virtual void stop() {}
};

class X2Source : public ScanSource {
public:
    explicit X2Source(const Config& c) : c_(c) {}
    bool start() override {
        std::string port = c_.port;
        laser_.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        std::string ignore;  // no angles masked by the SDK; we do our own filtering
        laser_.setlidaropt(LidarPropIgnoreArray, ignore.c_str(), ignore.size());
        int v = c_.baud;
        laser_.setlidaropt(LidarPropSerialBaudrate, &v, sizeof(int));
        v = TYPE_TRIANGLE;  // X2 measures by triangulation
        laser_.setlidaropt(LidarPropLidarType, &v, sizeof(int));
        v = YDLIDAR_TYPE_SERIAL;
        laser_.setlidaropt(LidarPropDeviceType, &v, sizeof(int));
        v = c_.sampleRateK;
        laser_.setlidaropt(LidarPropSampleRate, &v, sizeof(int));
        v = 4;
        laser_.setlidaropt(LidarPropAbnormalCheckCount, &v, sizeof(int));
        bool b = false;
        laser_.setlidaropt(LidarPropFixedResolution, &b, sizeof(bool));
        laser_.setlidaropt(LidarPropReversion, &b, sizeof(bool));
        b = true;  // counter-clockwise angles (left = positive), the ROS / maths convention
        laser_.setlidaropt(LidarPropInverted, &b, sizeof(bool));
        laser_.setlidaropt(LidarPropAutoReconnect, &b, sizeof(bool));
        b = c_.singleChannel;
        laser_.setlidaropt(LidarPropSingleChannel, &b, sizeof(bool));
        b = false;
        laser_.setlidaropt(LidarPropIntenstiy, &b, sizeof(bool));
        b = true;
        laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &b, sizeof(bool));
        float f = 180.0f;
        laser_.setlidaropt(LidarPropMaxAngle, &f, sizeof(float));
        f = -180.0f;
        laser_.setlidaropt(LidarPropMinAngle, &f, sizeof(float));
        f = c_.maxRangeM;
        laser_.setlidaropt(LidarPropMaxRange, &f, sizeof(float));
        f = c_.minRangeM;
        laser_.setlidaropt(LidarPropMinRange, &f, sizeof(float));
        f = 7.0f;  // X2 speed is set by motor voltage, not software; this is only a hint
        laser_.setlidaropt(LidarPropScanFrequency, &f, sizeof(float));

        if (!laser_.initialize()) {
            std::fprintf(stderr, "LiDAR initialize failed: %s\n", laser_.DescribeError());
            return false;
        }
        if (!laser_.turnOn()) {
            std::fprintf(stderr, "LiDAR turnOn failed: %s\n", laser_.DescribeError());
            return false;
        }
        return true;
    }
    bool next(std::vector<std::pair<float, float>>& out, float& scanHz) override {
        LaserScan scan;
        if (!laser_.doProcessSimple(scan)) return false;
        out.clear();
        for (const auto& p : scan.points) out.emplace_back(p.angle, p.range);
        scanHz = scan.scanFreq;
        return true;
    }
    void stop() override {
        laser_.turnOff();
        laser_.disconnecting();
    }

private:
    const Config& c_;
    CYdLidar laser_;
};

// A fake 2.4 m x 3 m room with a 20 cm box that drives towards the rover,
// pauses, then slides off to the side. Includes noise and random dust points.
class SimSource : public ScanSource {
public:
    bool start() override { return true; }
    bool next(std::vector<std::pair<float, float>>& out, float& scanHz) override {
        if (!fast_) std::this_thread::sleep_for(std::chrono::milliseconds(140));  // ~7 Hz like the X2
        scanHz = 7.0f;
        t_ += 0.14f;
        // Box centre over time (in the LiDAR frame, x ahead, y left).
        float bx, by = 0.0f;
        if (t_ < 2) bx = 1.6f;
        else if (t_ < 7) bx = 1.6f - (t_ - 2) * 0.28f;  // approaches to 0.2 m
        else if (t_ < 9) bx = 0.2f;                      // sits there
        else { bx = 0.2f; by = std::min(1.0f, (t_ - 9) * 0.35f); }  // slides away to the left
        if (t_ > 14) t_ = 0;
        const float half = 0.10f;
        out.clear();
        std::normal_distribution<float> noise(0.0f, 0.008f);
        std::uniform_real_distribution<float> uni(0.0f, 1.0f);
        for (int i = 0; i < 430; ++i) {
            float a = -kPi + i * (2 * kPi / 430);
            float dx = std::cos(a), dy = std::sin(a);
            float r = wall(dx, dy);
            float rb = rayBox(dx, dy, bx, by, half);
            if (rb > 0 && rb < r) r = rb;
            if (uni(rng_) < 0.03f) r = 0.0f;                        // dropouts (invalid 0 m)
            else if (uni(rng_) < 0.004f) r = 0.12f + uni(rng_) * 0.1f;  // lone dust point
            else r += noise(rng_);
            out.emplace_back(a, r);
        }
        return true;
    }

private:
    static float wall(float dx, float dy) {  // room: x in [-1.2, 1.8], y in [-1.2, 1.2]
        float best = 1e9f;
        if (dx > 1e-6f) best = std::min(best, 1.8f / dx);
        if (dx < -1e-6f) best = std::min(best, -1.2f / dx);
        if (dy > 1e-6f) best = std::min(best, 1.2f / dy);
        if (dy < -1e-6f) best = std::min(best, -1.2f / dy);
        return best;
    }
    static float rayBox(float dx, float dy, float cx, float cy, float h) {  // slab method
        float tmin = -1e9f, tmax = 1e9f;
        for (int k = 0; k < 2; ++k) {
            float d = k == 0 ? dx : dy, lo = (k == 0 ? cx : cy) - h, hi = (k == 0 ? cx : cy) + h;
            if (std::fabs(d) < 1e-6f) { if (0 < lo || 0 > hi) return -1; continue; }
            float t1 = lo / d, t2 = hi / d;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
        }
        return (tmax >= tmin && tmin > 0) ? tmin : -1;
    }
    float t_ = 0;
    std::mt19937 rng_{42};

public:
    bool fast_ = false;  // tests only: skip the 7 Hz wait
};

// ===========================================================================
// 3. Scan analysis: raw (angle, range) -> what's ahead, left and right
// ===========================================================================
struct ScanResult {
    std::vector<Pt> pts;        // valid points in the rover frame (for drawing)
    std::vector<Pt> corridor;   // points inside the forward corridor
    float aheadM = INFINITY;    // distance from the FRONT bumper to the obstacle (k-th nearest)
    float leftClearM = INFINITY, rightClearM = INFINITY;
    int valid = 0, raw = 0;
};

// Step 2b: the "lonely point" filter.
// Real objects are solid: they show up as several NEIGHBOURING points at almost
// the same distance. Dust, reflections and glitches are usually a single point
// on its own. Keep a point only if one of its 2 neighbours on either side (in
// angle order) is within neighbourTolM of it and close in angle.
// Analogy: one person saying they saw a ghost vs the whole group seeing it.
static std::vector<std::pair<float, float>> dropLonelyPoints(std::vector<std::pair<float, float>> v,
                                                             const Config& c) {
    std::sort(v.begin(), v.end());  // by angle
    const int n = int(v.size());
    std::vector<std::pair<float, float>> kept;
    kept.reserve(n);
    for (int i = 0; i < n; ++i) {
        bool supported = false;
        for (int d : {-2, -1, 1, 2}) {
            int j = (i + d + n) % n;  // wrap around at +/-180 deg
            float dAng = std::fabs(rad2deg(v[j].first - v[i].first));
            if (dAng > 180) dAng = 360 - dAng;
            if (dAng <= c.neighbourMaxDeg && std::fabs(v[j].second - v[i].second) < c.neighbourTolM) {
                supported = true;
                break;
            }
        }
        if (supported) kept.push_back(v[i]);
    }
    return kept;
}

static ScanResult analyze(const std::vector<std::pair<float, float>>& rawScan, const Config& c) {
    ScanResult r;
    r.raw = int(rawScan.size());
    // Step 2: drop invalid readings, then lonely points.
    std::vector<std::pair<float, float>> scan;
    for (const auto& pr : rawScan)
        if (pr.second >= c.minRangeM && pr.second <= c.maxRangeM) scan.push_back(pr);
    scan = dropLonelyPoints(scan, c);
    std::vector<float> ahead;
    for (const auto& [angRad, range] : scan) {
        // Step 3: rotate so 0 deg = rover forward; optionally mirror left/right.
        float deg = rad2deg(angRad) + c.forwardOffsetDeg;
        if (c.mirror) deg = -deg;
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        // Step 4: polar -> cartesian. x ahead, y left.
        float th = deg2rad(deg);
        Pt p{range * std::cos(th), range * std::sin(th), range, deg};
        // Step 5: ignore hits on the rover's own body.
        bool onRover = p.x > -c.lidarToRearM && p.x < c.lidarToFrontM && std::fabs(p.y) < c.halfWidthM;
        if (onRover) continue;
        r.valid++;
        r.pts.push_back(p);
        // Step 6: forward corridor = rectangle ahead of the bumper, rover width + margin.
        if (p.x > c.lidarToFrontM && std::fabs(p.y) < c.halfWidthM + c.marginM) {
            r.corridor.push_back(p);
            ahead.push_back(p.x - c.lidarToFrontM);
        }
        // Side sectors (bypass input): nearest point on each side.
        if (deg >= c.sideMinDeg && deg <= c.sideMaxDeg) r.leftClearM = std::min(r.leftClearM, range);
        if (deg <= -c.sideMinDeg && deg >= -c.sideMaxDeg) r.rightClearM = std::min(r.rightClearM, range);
    }
    // Step 7: k-th nearest, robust to a couple of stray points.
    if (int(ahead.size()) >= c.kthNearest) {
        std::nth_element(ahead.begin(), ahead.begin() + (c.kthNearest - 1), ahead.end());
        r.aheadM = ahead[c.kthNearest - 1];
    }
    return r;
}

// ===========================================================================
// 4. Stop latch with hysteresis
//    Analogy: a room thermostat that turns the heater ON at 18 C and OFF at
//    21 C. The gap stops it clicking on/off every second around one number.
// ===========================================================================
class StopLatch {
public:
    explicit StopLatch(const Config& c) : c_(c) {}
    bool update(float aheadM) {
        if (!blocked_) {
            stopStreak_ = aheadM < c_.stopDistM ? stopStreak_ + 1 : 0;
            if (stopStreak_ >= c_.scansToStop) { blocked_ = true; clearStreak_ = 0; }
        } else {
            clearStreak_ = aheadM > c_.resumeDistM ? clearStreak_ + 1 : 0;
            if (clearStreak_ >= c_.scansToResume) { blocked_ = false; stopStreak_ = 0; }
        }
        return blocked_;
    }

private:
    const Config& c_;
    bool blocked_ = false;
    int stopStreak_ = 0, clearStreak_ = 0;
};

// ===========================================================================
// 5. Radar view (top-down, rover at the centre, forward = UP)
// ===========================================================================
static cv::Mat drawRadar(const ScanResult& r, const Config& c, bool blocked, float hz) {
    const int S = 640, cx = S / 2, cy = S / 2 + 80;
    const float pxPerM = 180.0f;  // 1 m = 180 px
    cv::Mat img(S, S, CV_8UC3, cv::Scalar(25, 25, 25));
    auto toPx = [&](float x, float y) {  // x ahead -> up, y left -> left
        return cv::Point(int(cx - y * pxPerM), int(cy - x * pxPerM));
    };
    for (float m = 0.5f; m <= 2.5f; m += 0.5f) {
        cv::circle(img, {cx, cy}, int(m * pxPerM), cv::Scalar(60, 60, 60), 1);
        char b[16]; std::snprintf(b, sizeof(b), "%.1fm", m);
        cv::putText(img, b, {cx + 4, cy - int(m * pxPerM) - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(110, 110, 110), 1);
    }
    // Corridor (green when clear, red when blocked) and stop line.
    cv::Scalar zc = blocked ? cv::Scalar(60, 60, 230) : cv::Scalar(80, 200, 80);
    float w = c.halfWidthM + c.marginM;
    cv::rectangle(img, toPx(3.0f, w), toPx(c.lidarToFrontM, -w), zc, 1);
    cv::line(img, toPx(c.lidarToFrontM + c.stopDistM, w), toPx(c.lidarToFrontM + c.stopDistM, -w), zc, 2);
    cv::line(img, toPx(c.lidarToFrontM + c.resumeDistM, w), toPx(c.lidarToFrontM + c.resumeDistM, -w), zc, 1, cv::LINE_4);
    // Rover body.
    cv::rectangle(img, toPx(c.lidarToFrontM, c.halfWidthM), toPx(-c.lidarToRearM, -c.halfWidthM), cv::Scalar(200, 200, 200), 2);
    cv::arrowedLine(img, toPx(0, 0), toPx(c.lidarToFrontM + 0.05f, 0), cv::Scalar(200, 200, 200), 1);
    // Points.
    for (const auto& p : r.pts) cv::circle(img, toPx(p.x, p.y), 2, cv::Scalar(200, 180, 90), -1);
    for (const auto& p : r.corridor) cv::circle(img, toPx(p.x, p.y), 3, cv::Scalar(60, 60, 255), -1);
    // Text.
    char line[128];
    std::snprintf(line, sizeof(line), "%s   ahead %s", blocked ? "STOP" : "CLEAR",
                  std::isfinite(r.aheadM) ? cv::format("%.2f m", r.aheadM).c_str() : "--");
    cv::putText(img, line, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.8, zc, 2);
    std::snprintf(line, sizeof(line), "left %.2f m   right %.2f m   %.1f Hz   %d/%d pts",
                  std::isfinite(r.leftClearM) ? r.leftClearM : -1.f,
                  std::isfinite(r.rightClearM) ? r.rightClearM : -1.f, hz, r.valid, r.raw);
    cv::putText(img, line, {12, 54}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    return img;
}

// >>> CHANGED 30-Sep (4/8): this WHOLE section 6 is new (SerialPort + EspLink)
// ===========================================================================
// 6. Link to the ESP32 (only with --serial)
// ===========================================================================
// Write-only serial port (Linux). Same idea as in object_slowdown.cpp.
class SerialPort {
public:
    bool open(const std::string& dev, int baud) {
        fd_ = ::open(dev.c_str(), O_WRONLY | O_NOCTTY);
        if (fd_ < 0) return false;
        termios tty{};
        tcgetattr(fd_, &tty);
        cfmakeraw(&tty);  // send bytes exactly as given
        speed_t sp = baud == 9600 ? B9600 : baud == 57600 ? B57600 : B115200;
        cfsetospeed(&tty, sp);
        cfsetispeed(&tty, sp);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~HUPCL;  // don't drop DTR on close (can reset some ESP32 boards)
        return tcsetattr(fd_, TCSANOW, &tty) == 0;
    }
    void write(const std::string& s) {
        if (fd_ >= 0 && ::write(fd_, s.data(), s.size()) < 0) std::perror("ESP32 serial write");
    }
    ~SerialPort() {
        if (fd_ >= 0) ::close(fd_);
    }

private:
    int fd_ = -1;
};

// Heartbeat sender: its own thread sends the current command every 100 ms.
// Why a separate thread? The main loop waits ~143 ms for each LiDAR scan,
// which is SLOWER than the 100 ms heartbeat the ESP32 failsafe needs. So the
// main loop only updates "what to say"; this thread handles "when to say it".
// Analogy: a lighthouse keeps flashing on its own schedule; the keeper only
// changes the colour of the light.
// It also watches the LiDAR: if no scan has arrived for staleScanMs, it sends
// STOP ("no news is bad news"): a frozen or unplugged LiDAR must not leave an
// old "all clear" in charge.
class EspLink {
public:
    EspLink(SerialPort& port, const Config& c) : port_(port), c_(c) {
        lastScan_ = Clock::now();
        worker_ = std::thread([this] { loop(); });
    }
    ~EspLink() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        port_.write("S:0\n");  // leaving: ask for a stop, never leave it driving
    }
    // Called by the main loop after every scan.
    void report(bool blocked) {
        std::lock_guard<std::mutex> lk(m_);
        pct_ = blocked ? 0 : 100;
        lastScan_ = Clock::now();
    }
    bool stale() {
        std::lock_guard<std::mutex> lk(m_);
        return Clock::now() - lastScan_ > std::chrono::milliseconds(c_.staleScanMs);
    }

private:
    void loop() {
        while (running_) {
            int pct;
            {
                std::lock_guard<std::mutex> lk(m_);
                bool isStale = Clock::now() - lastScan_ > std::chrono::milliseconds(c_.staleScanMs);
                pct = isStale ? 0 : pct_;
            }
            port_.write("S:" + std::to_string(pct) + "\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(c_.sendEveryMs));
        }
    }
    SerialPort& port_;
    const Config& c_;
    std::thread worker_;
    std::mutex m_;
    int pct_ = 0;  // start at STOP until the first scan says otherwise
    Clock::time_point lastScan_;
    std::atomic<bool> running_{true};
};
// <<< END CHANGE (section 6)

// ===========================================================================
// main
// ===========================================================================
int main(int argc, char** argv) {
    Config cfg;
    bool sim = false, headless = false, calib = false;
    std::string espDev;  // CHANGED 30-Sep (5/8): new, ESP32 port name from --serial
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) cfg.port = argv[++i];
        else if (a == "--offset" && i + 1 < argc) cfg.forwardOffsetDeg = std::stof(argv[++i]);
        else if (a == "--mirror") cfg.mirror = true;
        else if (a == "--sim") sim = true;
        else if (a == "--headless") headless = true;
        else if (a == "--calib") calib = true;
        else if (a == "--serial" && i + 1 < argc) espDev = argv[++i];                   // CHANGED 30-Sep (5/8): new option
        else if (a == "--esp-baud" && i + 1 < argc) cfg.espBaud = std::stoi(argv[++i]);  // CHANGED 30-Sep (5/8): new option
    }

    std::setvbuf(stdout, nullptr, _IOLBF, 0);  // print each line immediately (useful when piping to a log)

    // >>> CHANGED 30-Sep (6/8): new block, open the ESP32 link and start the heartbeat
    // ESP32 link: checked BEFORE starting the LiDAR, so mistakes fail fast.
    if (!espDev.empty() && calib) {
        std::printf("Note: --serial ignored in --calib mode (the rover shouldn't drive while calibrating)\n");
        espDev.clear();
    }
    if (!espDev.empty() && !sim && espDev == cfg.port) {
        std::fprintf(stderr, "--serial and --port are both %s: the LiDAR and ESP32 need different ports\n",
                     espDev.c_str());
        return 1;
    }
    SerialPort espPort;
    std::unique_ptr<EspLink> esp;
    if (!espDev.empty()) {
        if (!espPort.open(espDev, cfg.espBaud)) {
            std::fprintf(stderr, "Could not open ESP32 port %s (wrong port? in dialout group?)\n", espDev.c_str());
            return 1;
        }
        esp = std::make_unique<EspLink>(espPort, cfg);  // starts sending S:0 until the first scan
        std::printf("ESP32 link open on %s: sending S:0 / S:100 every %d ms\n", espDev.c_str(), cfg.sendEveryMs);
    }
    // <<< END CHANGE

    os_init();  // installs the SDK's Ctrl+C handler -> os_isOk() becomes false
    std::unique_ptr<ScanSource> src;
    if (sim) src = std::make_unique<SimSource>();
    else src = std::make_unique<X2Source>(cfg);
    if (!src->start()) return 1;

    StopLatch latch(cfg);
    std::vector<std::pair<float, float>> scan;
    long n = 0;
    bool lastBlocked = false;
    while (os_isOk()) {
        float hz = 0;
        if (!src->next(scan, hz)) {
            std::fprintf(stderr, "No scan (check cable, port, 5 V power)\n");
            continue;
        }
        ScanResult r = analyze(scan, cfg);
        bool blocked = latch.update(r.aheadM);
        if (esp) esp->report(blocked);  // CHANGED 30-Sep (7/8): new, hand the decision to the heartbeat thread
        n++;

        if (calib) {
            // Put your hand ~20 cm in front of the rover: the angle printed is
            // where the LiDAR thinks "front" is. Use --offset <minus that angle>.
            const Pt* nearest = nullptr;
            for (const auto& p : r.pts) if (!nearest || p.range < nearest->range) nearest = &p;
            if (nearest && n % 3 == 0)
                std::printf("nearest point: %.2f m at %+.0f deg  (x %+.2f, y %+.2f)\n",
                            nearest->range, nearest->angleDeg, nearest->x, nearest->y);
        } else if (headless) {
            if (blocked != lastBlocked || n % 7 == 0)
                std::printf("scan %ld  %-5s  ahead %5s  left %.2f  right %.2f  %d pts\n", n,
                            blocked ? "STOP" : "CLEAR",
                            std::isfinite(r.aheadM) ? cv::format("%.2f", r.aheadM).c_str() : "--",
                            r.leftClearM, r.rightClearM, r.valid);
        }
        if (!headless) {
            cv::imshow("lidar_stop (forward = up)", drawRadar(r, cfg, blocked, hz));
            int k = cv::waitKey(1);
            if (k == 'q' || k == 27) break;
        }
        lastBlocked = blocked;
    }
    esp.reset();  // CHANGED 30-Sep (8/8): new, stop the heartbeat and send a final S:0 BEFORE the LiDAR shuts down
    src->stop();
    return 0;
}
