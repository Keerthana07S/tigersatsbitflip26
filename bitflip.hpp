// this is the code that directly interfaces with our hardware to write the bits. 
// Keerthana Srinivasan
// October 23, 2026
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace bf {

// I2C abstraction
class I2CBus {
public:
    virtual ~I2CBus() = default;
    // Plain write to a 7-bit address. Returns true on ACK.
    virtual bool write(uint8_t addr, const uint8_t* data, size_t len) = 0;
    // Write, repeated start, read.
    virtual bool writeRead(uint8_t addr, const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen) = 0;
};

enum class MemType { NVSRAM_CY14B101J2, EEPROM_M24M02, FRAM_FM24VN10 };

struct Device {
    std::string name;     // reference designator, e.g. "U1"
    MemType     type;
    uint8_t     muxChannel;   // TCA9548A channel
    uint8_t     baseAddr;     // 7-bit address with block-select bits = 0
    uint32_t    sizeBytes;
    double      boardX_mm;    // chip position on the PCB (fill in from layout)
    double      boardY_mm;
};

enum class EventKind {
    SEU,          // single bit flipped in one byte, confirmed by re-reads, scrubbed OK
    MBU,          // multiple bits in one byte, or adjacent bytes in the same scan
    STUCK,        // flip persisted after rewrite (hard error / possible TID damage)
    SEFI,         // large fraction of a chunk wrong: functional interrupt, not counted as SEUs
    READ_GLITCH,  // mismatch that vanished on re-read: false fault, filtered out
    BUS_ERROR     // NACK / I2C failure
};
const char* kindName(EventKind k);

struct FlipEvent {
    uint64_t  scan;
    double    t;          // unix time, seconds
    int       device;     // index into Monitor::devices()
    uint32_t  addr;
    uint8_t   expected;
    uint8_t   observed;
    uint8_t   mask;       // expected ^ observed (which bits flipped)
    EventKind kind;
    int       cluster;    // events sharing an id are spatially adjacent in one scan
    uint32_t  nBytes;     // bytes affected 
};

struct Config {
    uint8_t     muxAddr        = 0x70;     // TCA9548A with A0=A1=A2=GND
    uint32_t    chunk          = 256;      // read/write block (= M24M02 page size)
    int         rereads        = 3;        // re-reads used to reject false faults
    uint32_t    seed           = 0xB17F11B;
    double      sefiFraction   = 0.25;     // > this fraction of chunk wrong -> SEFI
    uint32_t    clusterDist    = 2;        // bytes
    std::string csvPath        = "bitflips.csv";
    std::string envCsvPath     = "scans.csv";
};

// Return extra variables to log every scan (voltage, current, temperature, dose rate...)
using EnvSampler = std::function<std::map<std::string, double>()>;

class Monitor {
public:
    Monitor(I2CBus& bus, std::vector<Device> devs, Config cfg = {});

    bool writePatterns();               // we can fill every device with the reference pattern
    std::vector<FlipEvent> scan();      // one full pass over all memories
    void setEnvSampler(EnvSampler s) { env_ = std::move(s); }

    // markov transition matrix for a device, row-major 4x4.
// note. I will edit this over the course of the semester as we make changed to the PCB. 
    // states: 0 clean, 1 SEU, 2 MBU, 3 fault (SEFI/STUCK/BUS)
    std::array<double, 16>   markov(int device) const;
    std::array<uint64_t, 16> markovCounts(int device) const { return trans_[device]; }

    const std::vector<Device>& devices() const { return devs_; }
    uint64_t scanCount() const { return scan_; }

    static uint8_t pattern(uint32_t seed, int dev, uint32_t addr);

private:
    uint8_t devAddr(const Device& d, uint32_t addr) const;
    bool selectChannel(uint8_t ch);
    bool readBlock(const Device& d, uint32_t addr, uint8_t* buf, size_t n);
    bool writeBlock(const Device& d, uint32_t addr, const uint8_t* buf, size_t n);
    void cluster(std::vector<FlipEvent>& ev);
    void updateMarkov(const std::vector<FlipEvent>& ev);
    void logEvents(const std::vector<FlipEvent>& ev);
    void logScan(double t0, double dur, const std::vector<FlipEvent>& ev);

    I2CBus&             bus_;
    std::vector<Device> devs_;
    Config              cfg_;
    EnvSampler          env_;
    uint64_t            scan_ = 0;
    int                 curCh_ = -1;
    int                 nextCluster_ = 0;
    std::vector<int>    lastState_;
    std::vector<std::array<uint64_t, 16>> trans_;
    bool                csvInit_ = false, envInit_ = false;
    std::vector<std::string> envKeys_;
};

// device table matching BitFlip.kicad_sch
std::vector<Device> bitflipBoard();

double nowSeconds();

} // namespace bf
