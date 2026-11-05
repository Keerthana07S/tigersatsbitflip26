// this is the code that primarily interfaces with our hardware 
// the purpose of this code is to feed the memory loggers with data and read back the data after "bit flips"
// the updated code for the Teensy will be included in the repository once it is commented. 
// Keerthana Srinivasan
// October 23, 2026

#include "bitflip.hpp"

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>

namespace bf {

// logger for time
double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

// returns the event type as a string (i.e., "SEU") which we use for analysis later
const char* kindName(EventKind k) {
    switch (k) {
        case EventKind::SEU: return "SEU";
        case EventKind::MBU: return "MBU";
        case EventKind::STUCK: return "STUCK";
        case EventKind::SEFI: return "SEFI";
        case EventKind::READ_GLITCH: return "READ_GLITCH";
        case EventKind::BUS_ERROR: return "BUS_ERROR";
    }
    return "?";
}

// the following is our board definition, which lays out all the PCB components. 
// Slave addresses (verify against datasheets):
//   CY14B101J2 memory: 1010 A2 A1 A16   FM24VN10: 1010 A2 A1 A16   M24M02: 1010 E2 A17 A16
// Channel 0 (SC0/SD0): U1 A2A1=00 -> 0x50, U2 =01 -> 0x52, U4 =10 -> 0x54, U5 =11 -> 0x56
// Channel 1 (SC1/SD1): U3 E2=0 -> 0x50..0x53
// Channel 2 (SC2/SD2): IC1 A2A1=00 -> 0x50, IC2 =01 -> 0x52
// Board X/Y are placeholders: fill in from your PCB layout.
std::vector<Device> bitflipBoard() {
    return {
        {"U1",  MemType::NVSRAM_CY14B101J2, 0, 0x50, 128 * 1024, 0, 0},
        {"U2",  MemType::NVSRAM_CY14B101J2, 0, 0x52, 128 * 1024, 0, 0},
        {"U4",  MemType::NVSRAM_CY14B101J2, 0, 0x54, 128 * 1024, 0, 0},
        {"U5",  MemType::NVSRAM_CY14B101J2, 0, 0x56, 128 * 1024, 0, 0},
        {"U3",  MemType::EEPROM_M24M02,     1, 0x50, 256 * 1024, 0, 0},
        {"IC1", MemType::FRAM_FM24VN10,     2, 0x50, 128 * 1024, 0, 0},
        {"IC2", MemType::FRAM_FM24VN10,     2, 0x52, 128 * 1024, 0, 0},
    };
}

// ---------------- Monitor ----------------
Monitor::Monitor(I2CBus& bus, std::vector<Device> devs, Config cfg)
    : bus_(bus), devs_(std::move(devs)), cfg_(std::move(cfg)),
      lastState_(devs_.size(), -1), trans_(devs_.size()) {
    for (auto& t : trans_) t.fill(0);
}

// pattern() is the answer key we follow for the byte that should be at each address. 
// address-dependent pseudo-random pattern: ~50% ones (balanced 0->1 / 1->0 stats),
// and it also catches address-line faults that a fixed checkerboard would miss.
uint8_t Monitor::pattern(uint32_t seed, int dev, uint32_t a) {
    uint32_t x = a * 0x9E3779B1u ^ static_cast<uint32_t>(dev) * 0x85EBCA77u ^ seed;
    x ^= x >> 15; x *= 0x2C1B3C6Du;
    x ^= x >> 12; x *= 0x297A2D39u;
    x ^= x >> 15;
    return static_cast<uint8_t>(x);
}

// for now, we are only sending 2-byte memory addresses, so any extra address bits need to be moved to the I2C address
uint8_t Monitor::devAddr(const Device& d, uint32_t addr) const {
    uint32_t blockMask = (d.sizeBytes - 1) >> 16;  
    return static_cast<uint8_t>(d.baseAddr | ((addr >> 16) & blockMask));
}

// opens the channel we want
bool Monitor::selectChannel(uint8_t ch) {
    if (curCh_ == ch) return true;
    uint8_t v = static_cast<uint8_t>(1u << ch);
    if (!bus_.write(cfg_.muxAddr, &v, 1)) { curCh_ = -1; return false; }
    curCh_ = ch;
    return true;
}

// read bytes after requesting the address
bool Monitor::readBlock(const Device& d, uint32_t addr, uint8_t* buf, size_t n) {
    if (!selectChannel(d.muxChannel)) return false;
    uint8_t w[2] = {static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr)};
    return bus_.writeRead(devAddr(d, addr), w, 2, buf, n);
}

// write bytes after requesting the address.
// note that I read U3 takes longer to save data so there is a separate part for that where we will wait longer for the bytes to register before moving on
bool Monitor::writeBlock(const Device& d, uint32_t addr, const uint8_t* buf, size_t n) {
    if (!selectChannel(d.muxChannel)) return false;
    std::vector<uint8_t> tx(n + 2);
    tx[0] = static_cast<uint8_t>(addr >> 8);
    tx[1] = static_cast<uint8_t>(addr);
    std::memcpy(tx.data() + 2, buf, n);
    uint8_t a = devAddr(d, addr);
    if (!bus_.write(a, tx.data(), tx.size())) return false;
    if (d.type == MemType::EEPROM_M24M02) {
        for (int i = 0; i < 40; ++i) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            if (bus_.write(a, tx.data(), 2)) return true;
        }
        return false;
    }
    return true;   
}

// writing the initial pattern
bool Monitor::writePatterns() {
    std::vector<uint8_t> buf(cfg_.chunk);
    bool ok = true;
    for (int di = 0; di < static_cast<int>(devs_.size()); ++di) {
        const Device& d = devs_[di];
        curCh_ = -1;
        for (uint32_t a = 0; a < d.sizeBytes; a += cfg_.chunk) {
            uint32_t n = std::min(cfg_.chunk, d.sizeBytes - a);
            for (uint32_t i = 0; i < n; ++i) buf[i] = pattern(cfg_.seed, di, a + i);
            if (!writeBlock(d, a, buf.data(), n)) { ok = false; break; }
        }
    }
    return ok;
}

// 1. read data and compare with pattern
// 2. to filter out false faults, re-read 3 types whenever pattern doesn't match
// 3. if more than a quarter of the pattern is wrong, then it is likely a SEFI or the entire chip malfunctioning
// 4. if one bit is wrong, SEU. if more than one bit is wrong, MEU.
// 5. check if the bit can be corrected. rewrite the pattern, and if there is still an issue we mark the chip as STUCK.
std::vector<FlipEvent> Monitor::scan() {
    ++scan_;
    std::vector<FlipEvent> ev;
    const double t0 = nowSeconds();
    std::vector<uint8_t> buf(cfg_.chunk), exp(cfg_.chunk), rr(cfg_.chunk);

    for (int di = 0; di < static_cast<int>(devs_.size()); ++di) {
        const Device& d = devs_[di];
        curCh_ = -1;   // always reselect: the mux register itself can be upset
        for (uint32_t a = 0; a < d.sizeBytes; a += cfg_.chunk) {
            const uint32_t n = std::min(cfg_.chunk, d.sizeBytes - a);
            const double t = nowSeconds();
            auto add = [&](uint32_t addr, uint8_t e, uint8_t o, EventKind k, uint32_t nb = 1) {
                ev.push_back({scan_, t, di, addr, e, o, static_cast<uint8_t>(e ^ o), k, -1, nb});
            };

            if (!readBlock(d, a, buf.data(), n)) { add(a, 0, 0, EventKind::BUS_ERROR); curCh_ = -1; continue; }
            for (uint32_t i = 0; i < n; ++i) exp[i] = pattern(cfg_.seed, di, a + i);

            std::vector<uint32_t> bad;
            for (uint32_t i = 0; i < n; ++i) if (buf[i] != exp[i]) bad.push_back(i);
            if (bad.empty()) continue;

            // here, we use the fact that a real upset would read back the same each time, otherwise it is just noise. 
            std::vector<char> consistent(bad.size(), 1);
            bool busFail = false;
            for (int r = 0; r < cfg_.rereads && !busFail; ++r) {
                if (!readBlock(d, a, rr.data(), n)) { busFail = true; break; }
                for (size_t k = 0; k < bad.size(); ++k)
                    if (rr[bad[k]] != buf[bad[k]]) consistent[k] = 0;
            }
            if (busFail) { add(a, 0, 0, EventKind::BUS_ERROR); curCh_ = -1; continue; }

            std::vector<uint32_t> real;
            for (size_t k = 0; k < bad.size(); ++k) {
                if (consistent[k]) real.push_back(bad[k]);
                else add(a + bad[k], exp[bad[k]], buf[bad[k]], EventKind::READ_GLITCH);
            }
            if (real.empty()) continue;

            // if there are too many errors at once, it is a SEFI.
            if (real.size() > cfg_.sefiFraction * n) {
                add(a, 0, 0, EventKind::SEFI, static_cast<uint32_t>(real.size()));
                writeBlock(d, a, exp.data(), n);
                continue;
            }

            // try to write the pattern again. 
            for (uint32_t i : real) {
                uint8_t m = exp[i] ^ buf[i];
                EventKind k = std::bitset<8>(m).count() > 1 ? EventKind::MBU : EventKind::SEU;
                uint8_t v = 0;
                bool okW = writeBlock(d, a + i, &exp[i], 1);
                bool okR = readBlock(d, a + i, &v, 1);
                if (okW && okR && v != exp[i]) k = EventKind::STUCK;
                add(a + i, exp[i], buf[i], k);
            }
        }
    }

    cluster(ev);
    updateMarkov(ev);
    logEvents(ev);
    logScan(t0, nowSeconds() - t0, ev);
    return ev;
}

// group flips that are close in address within the same device and scan.
// note: logical address adjacency is only a proxy for physical cell adjacency. we need to actively reference our PCB and be more critical for this part
// I will try to come up with better logic for this part later. 
void Monitor::cluster(std::vector<FlipEvent>& ev) {
    std::vector<size_t> idx;
    for (size_t i = 0; i < ev.size(); ++i)
        if (ev[i].kind == EventKind::SEU || ev[i].kind == EventKind::MBU || ev[i].kind == EventKind::STUCK)
            idx.push_back(i);
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
        return ev[a].device != ev[b].device ? ev[a].device < ev[b].device : ev[a].addr < ev[b].addr;
    });

    std::vector<std::vector<size_t>> groups;
    for (size_t j = 0; j < idx.size(); ++j) {
        const FlipEvent& e = ev[idx[j]];
        if (j > 0) {
            const FlipEvent& p = ev[idx[j - 1]];
            if (p.device == e.device && e.addr - p.addr <= cfg_.clusterDist) { groups.back().push_back(idx[j]); continue; }
        }
        groups.push_back({idx[j]});
    }
    for (auto& g : groups) {
        int id = nextCluster_++;
        for (size_t i : g) {
            ev[i].cluster = id;
            if (g.size() > 1 && ev[i].kind == EventKind::SEU) ev[i].kind = EventKind::MBU;
        }
    }
}

// the purpose of the markov chain is we can propogate through faults and evaluate the probability a chip actually has a fault
// otherwise, we may mistake a propogated SEU to be an MEU. 
// each type of event is associated with a state. the markov chain will analyze the state propogation of each chip and from here, we can form a probability distribution. 
void Monitor::updateMarkov(const std::vector<FlipEvent>& ev) {
    std::vector<int> state(devs_.size(), 0);
    for (const auto& e : ev) {
        int s = 0;
        switch (e.kind) {
            case EventKind::SEU: s = 1; break;
            case EventKind::MBU: s = 2; break;
            case EventKind::STUCK: case EventKind::SEFI: case EventKind::BUS_ERROR: s = 3; break;
            case EventKind::READ_GLITCH: s = 0; break;   // false faults don't change state
        }
        state[e.device] = std::max(state[e.device], s);
    }
    for (size_t d = 0; d < devs_.size(); ++d) {
        if (lastState_[d] >= 0) trans_[d][lastState_[d] * 4 + state[d]]++;
        lastState_[d] = state[d];
    }
}

std::array<double, 16> Monitor::markov(int device) const {
    std::array<double, 16> P{};
    const auto& c = trans_[device];
    for (int i = 0; i < 4; ++i) {
        uint64_t row = c[i * 4] + c[i * 4 + 1] + c[i * 4 + 2] + c[i * 4 + 3];
        for (int j = 0; j < 4; ++j) P[i * 4 + j] = row ? double(c[i * 4 + j]) / row : 0.0;
    }
    return P;
}

// we can save the data to bitflips.csv
static bool fileHasData(const std::string& p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec) && std::filesystem::file_size(p, ec) > 0;
}

void Monitor::logEvents(const std::vector<FlipEvent>& ev) {
    std::ofstream f(cfg_.csvPath, std::ios::app);
    if (!csvInit_) {
        if (!fileHasData(cfg_.csvPath))
            f << "scan,time,device,type,mux_ch,addr,expected,observed,mask,nbits,bit_positions,"
                 "dir_0to1,dir_1to0,kind,cluster,nbytes,board_x_mm,board_y_mm\n";
        csvInit_ = true;
    }
    for (const auto& e : ev) {
        const Device& d = devs_[e.device];
        std::string bits;
        int up = 0, down = 0;
        for (int b = 0; b < 8; ++b) if (e.mask >> b & 1) {
            if (!bits.empty()) bits += ';';
            bits += std::to_string(b);
            (e.observed >> b & 1) ? ++up : ++down;
        }
        f << e.scan << ',' << std::fixed << e.t << ',' << d.name << ',' << int(d.type) << ','
          << int(d.muxChannel) << ',' << e.addr << ',' << int(e.expected) << ',' << int(e.observed) << ','
          << int(e.mask) << ',' << std::bitset<8>(e.mask).count() << ',' << bits << ','
          << up << ',' << down << ',' << kindName(e.kind) << ',' << e.cluster << ',' << e.nBytes << ','
          << d.boardX_mm << ',' << d.boardY_mm << '\n';
    }
}

void Monitor::logScan(double t0, double dur, const std::vector<FlipEvent>& ev) {
    std::map<std::string, double> env;
    if (env_) env = env_();
    std::ofstream f(cfg_.envCsvPath, std::ios::app);
    if (!envInit_) {
        for (auto& kv : env) envKeys_.push_back(kv.first);
        if (!fileHasData(cfg_.envCsvPath)) {
            f << "scan,time,duration_s,seu,mbu,stuck,sefi,glitch,bus_err";
            for (auto& k : envKeys_) f << ',' << k;
            f << '\n';
        }
        envInit_ = true;
    }
    int n[6] = {0};
    for (const auto& e : ev) n[int(e.kind)]++;
    f << scan_ << ',' << std::fixed << t0 << ',' << dur;
    for (int i = 0; i < 6; ++i) f << ',' << n[i];
    for (auto& k : envKeys_) { auto it = env.find(k); f << ',' << (it != env.end() ? it->second : NAN); }
    f << '\n';
}

} // namespace bf
