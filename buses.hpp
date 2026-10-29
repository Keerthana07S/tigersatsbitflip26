#pragma once
#include "bitflip.hpp"
#include <random>
#include <stdexcept>
#include <vector>

#ifdef __linux__
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace bf {

#ifdef __linux__
// Raspberry Pi / any Linux host: /dev/i2c-1 etc.
class LinuxI2C : public I2CBus {
    int fd_ = -1;
public:
    explicit LinuxI2C(const char* dev) {
        fd_ = ::open(dev, O_RDWR);
        if (fd_ < 0) throw std::runtime_error(std::string("cannot open ") + dev);
    }
    ~LinuxI2C() override { if (fd_ >= 0) ::close(fd_); }

    bool write(uint8_t a, const uint8_t* d, size_t n) override {
        i2c_msg m{a, 0, static_cast<__u16>(n), const_cast<uint8_t*>(d)};
        i2c_rdwr_ioctl_data x{&m, 1};
        return ioctl(fd_, I2C_RDWR, &x) >= 0;
    }
    bool writeRead(uint8_t a, const uint8_t* w, size_t wl, uint8_t* r, size_t rl) override {
        i2c_msg m[2] = {{a, 0, static_cast<__u16>(wl), const_cast<uint8_t*>(w)},
                        {a, I2C_M_RD, static_cast<__u16>(rl), r}};
        i2c_rdwr_ioctl_data x{m, 2};
        return ioctl(fd_, I2C_RDWR, &x) >= 0;
    }
};
#endif

// Simulated board: emulates the mux + all 7 memories, injects real upsets
// and transient read glitches so you can test the pipeline with no hardware.
class SimBus : public I2CBus {
public:
    SimBus(std::vector<Device> devs, uint8_t mux = 0x70, double glitchPerByte = 1e-6, uint32_t seed = 1)
        : devs_(std::move(devs)), mux_(mux), glitch_(glitchPerByte), rng_(seed) {
        for (auto& d : devs_) mem_.emplace_back(d.sizeBytes, 0);
    }

    // Flip n random bits; ~10% become 2-bit adjacent upsets
    void injectUpsets(int n) {
        std::uniform_int_distribution<int> dev(0, int(devs_.size()) - 1), bit(0, 7);
        std::uniform_real_distribution<double> u(0, 1);
        for (int i = 0; i < n; ++i) {
            int d = dev(rng_);
            uint32_t a = std::uniform_int_distribution<uint32_t>(0, devs_[d].sizeBytes - 2)(rng_);
            int b = bit(rng_);
            mem_[d][a] ^= uint8_t(1u << b);
            if (u(rng_) < 0.1) mem_[d][a + 1] ^= uint8_t(1u << b);
        }
    }

    bool write(uint8_t a, const uint8_t* d, size_t n) override {
        if (a == mux_) { if (n != 1) return false; ch_ = d[0]; return true; }
        int di; uint32_t blk;
        if (!target(a, di, blk)) return false;
        if (n < 2) return true;
        uint32_t p = blk << 16 | uint32_t(d[0]) << 8 | d[1];
        for (size_t i = 2; i < n; ++i) mem_[di][(p + i - 2) % devs_[di].sizeBytes] = d[i];
        return true;
    }
    bool writeRead(uint8_t a, const uint8_t* w, size_t wl, uint8_t* r, size_t rl) override {
        int di; uint32_t blk;
        if (!target(a, di, blk) || wl < 2) return false;
        uint32_t p = blk << 16 | uint32_t(w[0]) << 8 | w[1];
        std::uniform_real_distribution<double> u(0, 1);
        for (size_t i = 0; i < rl; ++i) {
            r[i] = mem_[di][(p + i) % devs_[di].sizeBytes];
            if (u(rng_) < glitch_) r[i] ^= uint8_t(1u << (rng_() % 8));   // transient, not stored
        }
        return true;
    }

private:
    bool target(uint8_t a, int& di, uint32_t& blk) const {
        for (size_t i = 0; i < devs_.size(); ++i) {
            const Device& d = devs_[i];
            uint8_t mask = uint8_t((d.sizeBytes - 1) >> 16);
            if ((ch_ >> d.muxChannel & 1) && (a & ~mask) == d.baseAddr) {
                di = int(i); blk = a & mask; return true;
            }
        }
        return false;
    }
    std::vector<Device> devs_;
    std::vector<std::vector<uint8_t>> mem_;
    uint8_t mux_, ch_ = 0;
    double glitch_;
    std::mt19937 rng_;
};

} // namespace bf
