#pragma once
#include <cstddef>

#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#elif defined(__linux__)
#include <cstdio>
#endif

// Returns the current process's resident memory usage in bytes, or 0 if it
// could not be determined on this platform. This is a real OS-level
// measurement - a separate, complementary number to
// LPAStarPlanner::approximateMemoryBytes(), which estimates memory
// analytically by summing container sizes instead of asking the OS.
inline std::size_t currentProcessRSSBytes() {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS info;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &info, sizeof(info))) {
        return static_cast<std::size_t>(info.WorkingSetSize);
    }
    return 0;
#elif defined(__linux__)
    FILE* f = std::fopen("/proc/self/status", "r");
    if (!f) return 0;
    char line[256];
    std::size_t rssKb = 0;
    while (std::fgets(line, sizeof(line), f)) {
        if (std::sscanf(line, "VmRSS: %zu kB", &rssKb) == 1) break;
    }
    std::fclose(f);
    return rssKb * 1024;
#else
    return 0; // unsupported platform - callers should handle a 0 return gracefully
#endif
}