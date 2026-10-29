// bitflip_monitor --sim                 run against simulated board
// bitflip_monitor --dev /dev/i2c-1      run on hardware
// options: --scans N (0 = forever), --interval SECONDS, --no-fill (keep existing contents)
#include "bitflip.hpp"
#include "buses.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

static std::atomic<bool> g_run{true};

int main(int argc, char** argv) {
    bool sim = false, fill = true;
    const char* dev = "/dev/i2c-1";
    long scans = 0;
    double interval = 1.0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--sim")) sim = true;
        else if (!strcmp(argv[i], "--no-fill")) fill = false;
        else if (!strcmp(argv[i], "--dev") && i + 1 < argc) dev = argv[++i];
        else if (!strcmp(argv[i], "--scans") && i + 1 < argc) scans = atol(argv[++i]);
        else if (!strcmp(argv[i], "--interval") && i + 1 < argc) interval = atof(argv[++i]);
    }
    std::signal(SIGINT, [](int) { g_run = false; });

    auto board = bf::bitflipBoard();
    std::unique_ptr<bf::I2CBus> bus;
    bf::SimBus* simBus = nullptr;
    if (sim) { auto s = std::make_unique<bf::SimBus>(board); simBus = s.get(); bus = std::move(s); }
#ifdef __linux__
    else bus = std::make_unique<bf::LinuxI2C>(dev);
#else
    else { fprintf(stderr, "hardware bus only implemented for Linux\n"); return 1; }
#endif

    bf::Monitor mon(*bus, board);

    // Hook your voltage/current/temperature/dose sensors here (e.g. INA219 over I2C or an ADC).
    mon.setEnvSampler([sim]() -> std::map<std::string, double> {
        if (sim) return {{"vcc_V", 3.3 + 0.01 * std::sin(bf::nowSeconds())}, {"icc_mA", 4.2}, {"temp_C", 25.0}};
        return {{"vcc_V", NAN}, {"icc_mA", NAN}, {"temp_C", NAN}, {"dose_rate", NAN}};
    });

    if (fill) {
        printf("Writing reference patterns...\n");
        if (!mon.writePatterns()) fprintf(stderr, "WARNING: some pattern writes failed\n");
    }

    while (g_run && (scans == 0 || long(mon.scanCount()) < scans)) {
        if (simBus) simBus->injectUpsets(3);
        auto ev = mon.scan();
        int n[6] = {0};
        for (auto& e : ev) n[int(e.kind)]++;
        printf("scan %llu: SEU %d  MBU %d  STUCK %d  SEFI %d  | filtered glitches %d  bus errors %d\n",
               (unsigned long long)mon.scanCount(), n[0], n[1], n[2], n[3], n[4], n[5]);
        fflush(stdout);
        if (g_run) std::this_thread::sleep_for(std::chrono::duration<double>(interval));
    }

    const char* st[4] = {"clean", "SEU", "MBU", "fault"};
    for (size_t d = 0; d < mon.devices().size(); ++d) {
        auto P = mon.markov(int(d));
        printf("\nMarkov transition matrix %s (rows: from, cols: to)\n       ", mon.devices()[d].name.c_str());
        for (auto s : st) printf("%8s", s);
        for (int i = 0; i < 4; ++i) {
            printf("\n%7s", st[i]);
            for (int j = 0; j < 4; ++j) printf("%8.3f", P[i * 4 + j]);
        }
        printf("\n");
    }
    return 0;
}
