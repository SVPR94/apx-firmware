// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <functional>
#include <optional>
#include "apx/kernel/mutex.hpp"

// Namespace kept as apx::gcs (its original home before this file moved to
// modules/params/) rather than renamed to apx::params, to avoid rippling a
// cosmetic-only change through every existing call site (modules/gcs/*.cpp
// use it unqualified from inside namespace apx::gcs already) - what
// actually matters for the modes/gcs layering rule is the CMake target
// graph (apx_params has no dependency on apx_gcs or apx_flight_modes, and
// both of those depend on apx_params - see modules/params/CMakeLists.txt),
// not what this namespace happens to be spelled.
namespace apx::gcs {

enum class ParamType : uint8_t {
    INT32 = 0,
    FLOAT = 1,
    BOOL = 2
};

struct ParamValue {
    union {
        int32_t i;
        float f;
        bool b;
    };
    ParamType type;

    ParamValue() : i(0), type(ParamType::INT32) {}
    ParamValue(int32_t val) : i(val), type(ParamType::INT32) {}
    ParamValue(float val) : f(val), type(ParamType::FLOAT) {}
    ParamValue(bool val) : b(val), type(ParamType::BOOL) {}
};

struct ParamDefinition {
    char name[17]; // Up to 16 chars + null terminator
    ParamType type;
    ParamValue default_value;
    ParamValue min_value;
    ParamValue max_value;
    const char* description;
};

using ParamChangeCallback = std::function<void(const std::string&, const ParamValue&)>;

class ParamManager {
public:
    static ParamManager& get_instance();

    ParamManager(const ParamManager&) = delete;
    ParamManager& operator=(const ParamManager&) = delete;

    void register_param(const ParamDefinition& def);
    
    bool set_float(const std::string& name, float val);
    bool set_int(const std::string& name, int32_t val);
    bool set_bool(const std::string& name, bool val);

    std::optional<float> get_float(const std::string& name) const;
    std::optional<int32_t> get_int(const std::string& name) const;
    std::optional<bool> get_bool(const std::string& name) const;

    // Convenience for callers (flight_control gain/limit setup) that want
    // "the param's value, or this compiled-in fallback if it isn't
    // registered/loaded" without an if/has_value() at every call site.
    // Only actually falls back in practice if called before init_defaults()
    // has registered the name in question.
    float get_float_or(const std::string& name, float fallback) const;
    int32_t get_int_or(const std::string& name, int32_t fallback) const;

    std::optional<ParamValue> get_param(const std::string& name) const;
    std::vector<std::string> get_all_param_names() const;
    const ParamDefinition* get_param_def(const std::string& name) const;

    void add_change_callback(ParamChangeCallback cb);

    bool load_from_storage();
    bool save_to_storage();

    void init_defaults();

private:
    ParamManager() = default;
    ~ParamManager() = default;

    // apx::kernel::Mutex, not std::mutex: this toolchain's arm-none-eabi
    // libstdc++ is a single-threaded (no gthreads) build, so <mutex>
    // compiles but doesn't actually define std::mutex at all - confirmed
    // by an STM32H7 build attempt, not assumed. apx::kernel::Mutex (see
    // kernel/ports/<os>/mutex.cpp) has a hosted (std::mutex, SITL/Linux),
    // Zephyr (k_mutex) and bare-metal (IRQ-masking) implementation.
    mutable apx::kernel::Mutex m_mutex;
    std::unordered_map<std::string, ParamDefinition> m_definitions;
    std::unordered_map<std::string, ParamValue> m_values;
    std::vector<ParamChangeCallback> m_callbacks;

    void notify_callbacks(const std::string& name, const ParamValue& val);
};

#define APX_DEFINE_PARAM_INT(_name, _default, _min, _max, _desc) \
    namespace { \
        struct ParamRegister_##_name { \
            ParamRegister_##_name() { \
                apx::gcs::ParamDefinition def{}; \
                std::strncpy(def.name, #_name, 16); \
                def.name[16] = '\0'; \
                def.type = apx::gcs::ParamType::INT32; \
                def.default_value = apx::gcs::ParamValue(static_cast<int32_t>(_default)); \
                def.min_value = apx::gcs::ParamValue(static_cast<int32_t>(_min)); \
                def.max_value = apx::gcs::ParamValue(static_cast<int32_t>(_max)); \
                def.description = _desc; \
                apx::gcs::ParamManager::get_instance().register_param(def); \
            } \
        } param_register_##_name; \
    }

#define APX_DEFINE_PARAM_FLOAT(_name, _default, _min, _max, _desc) \
    namespace { \
        struct ParamRegister_##_name { \
            ParamRegister_##_name() { \
                apx::gcs::ParamDefinition def{}; \
                std::strncpy(def.name, #_name, 16); \
                def.name[16] = '\0'; \
                def.type = apx::gcs::ParamType::FLOAT; \
                def.default_value = apx::gcs::ParamValue(static_cast<float>(_default)); \
                def.min_value = apx::gcs::ParamValue(static_cast<float>(_min)); \
                def.max_value = apx::gcs::ParamValue(static_cast<float>(_max)); \
                def.description = _desc; \
                apx::gcs::ParamManager::get_instance().register_param(def); \
            } \
        } param_register_##_name; \
    }

} // namespace apx::gcs
