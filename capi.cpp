// C ABI so Python (ctypes) and Julia (ccall) can drive the monitor directly.
// I used Claude for snippets and filled in some parts myself once I learned more, b/c this was my first time integrating Julia, Python, and C++.
// Keerthana Srinivasan 
// October 23, 2026
#include "bitflip.hpp"
#include "buses.hpp"
#include <memory>

struct BfHandle {
    std::unique_ptr<bf::I2CBus> bus;
    bf::SimBus* sim = nullptr;
    std::unique_ptr<bf::Monitor> mon;
    std::vector<bf::FlipEvent> last;
};

extern "C" {

struct bf_event_c {
    uint64_t scan;
    double   t;
    int32_t  device;
    uint32_t addr;
    uint8_t  expected, observed, mask, kind;  // kind: 0 SEU 1 MBU 2 STUCK 3 SEFI 4 GLITCH 5 BUS
    int32_t  cluster;
    uint32_t nbytes;
};

void* bf_open(const char* i2cdev, int simulate) {
    try {
        auto h = new BfHandle;
        auto board = bf::bitflipBoard();
        if (simulate) { auto s = std::make_unique<bf::SimBus>(board); h->sim = s.get(); h->bus = std::move(s); }
#ifdef __linux__
        else h->bus = std::make_unique<bf::LinuxI2C>(i2cdev);
#else
        else { delete h; return nullptr; }
#endif
        h->mon = std::make_unique<bf::Monitor>(*h->bus, board);
        return h;
    } catch (...) { return nullptr; }
}

void bf_close(void* h) { delete static_cast<BfHandle*>(h); }

int bf_num_devices(void* h) { return int(static_cast<BfHandle*>(h)->mon->devices().size()); }

const char* bf_device_name(void* h, int i) { return static_cast<BfHandle*>(h)->mon->devices()[i].name.c_str(); }

int bf_write_patterns(void* h) { return static_cast<BfHandle*>(h)->mon->writePatterns() ? 1 : 0; }

int bf_scan(void* hp) {
    auto h = static_cast<BfHandle*>(hp);
    h->last = h->mon->scan();
    return int(h->last.size());
}

int bf_get_event(void* hp, int i, bf_event_c* out) {
    auto h = static_cast<BfHandle*>(hp);
    if (i < 0 || i >= int(h->last.size())) return 0;
    const auto& e = h->last[i];
    *out = {e.scan, e.t, e.device, e.addr, e.expected, e.observed, e.mask,
            uint8_t(e.kind), e.cluster, e.nBytes};
    return 1;
}

// out: 16 doubles (probabilities) or 16 uint64 counts, row-major 4x4
void bf_markov(void* h, int device, double* out) {
    auto P = static_cast<BfHandle*>(h)->mon->markov(device);
    for (int i = 0; i < 16; ++i) out[i] = P[i];
}
void bf_markov_counts(void* h, int device, uint64_t* out) {
    auto C = static_cast<BfHandle*>(h)->mon->markovCounts(device);
    for (int i = 0; i < 16; ++i) out[i] = C[i];
}

void bf_sim_inject(void* hp, int n) {
    auto h = static_cast<BfHandle*>(hp);
    if (h->sim) h->sim->injectUpsets(n);
}

} // extern "C"
