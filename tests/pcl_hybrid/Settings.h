// Simple global settings holder for runtime configuration
#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
using json = nlohmann::json;

// Global settings JSON, readable/writable by all modules
extern json _SETTINGS;

// Helper to load settings from a file into _SETTINGS (returns true on success)
inline bool LoadSettingsFromFile(const std::string &path) {
    try {
        std::ifstream ifs(path);
        if (!ifs) return false;
        json j = json::parse(ifs);
        _SETTINGS = j;
        return true;
    } catch (...) {
        return false;
    }
}
