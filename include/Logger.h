#ifndef GLOBAL_LOGGER_H
#define GLOBAL_LOGGER_H

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>
#include <mutex>

class GlobalLogger {
public:
    static GlobalLogger& getInstance() {
        static GlobalLogger instance;
        return instance;
    }

    void logTime(const std::string& name, long long duration) {
        std::lock_guard<std::mutex> lock(mutex);
        auto& data = times[name];
        data.first += duration;
        data.second += 1;
    }

    void printAverages() const {
        std::lock_guard<std::mutex> lock(mutex);
        auto orderingFunction = [](const std::pair<std::string, std::pair<long long, long long>>& a, const std::pair<std::string, std::pair<long long, long long>>& b) {
            return a.second.first > b.second.first;
        };
        auto timesVector = std::vector<std::pair<std::string, std::pair<long long, long long>>>(times.begin(), times.end());
        sort(timesVector.begin(), timesVector.end(), orderingFunction);
        std::cout << "Ordered from the highest time consuming to the lowest:" << std::endl;
        for (const auto& entry : timesVector) {
            const auto& name = entry.first;
            const auto& data = entry.second;
            long long totalDuration = data.first;
            long long count = data.second;
            std::cout << name << " average time: " << (totalDuration / count) << " ms" << std::endl;
            std::cout << name << " total call  : " << count << std::endl;
            std::cout << name << " total time  : " << totalDuration << " ms" << std::endl;
        }
    }

private:
    GlobalLogger() = default;
    ~GlobalLogger() = default;
    GlobalLogger(const GlobalLogger&) = delete;
    GlobalLogger& operator=(const GlobalLogger&) = delete;

    mutable std::mutex mutex;
    std::unordered_map<std::string, std::pair<long long, long long>> times;
};

class ScopedLogger {
public:
    ScopedLogger(const std::string& name) : name(name), start(std::chrono::high_resolution_clock::now()) {}

    ~ScopedLogger() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        GlobalLogger::getInstance().logTime(name, duration);
    }

private:
    std::string name;
    std::chrono::high_resolution_clock::time_point start;
};

#endif // GLOBAL_LOGGER_H