// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include "driver_base.hpp"
#include <unordered_map>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace apx::drivers {

class DriverRegistry {
public:
    using FactoryFn = std::function<std::unique_ptr<DriverBase>()>;

    static DriverRegistry& get_instance() {
        static DriverRegistry inst;
        return inst;
    }

    void register_driver(const std::string& name, FactoryFn factory) {
        m_factories[name] = std::move(factory);
    }

    std::unique_ptr<DriverBase> create(const std::string& name) const {
        auto it = m_factories.find(name);
        if (it == m_factories.end()) return nullptr;
        return it->second();
    }

    std::vector<std::string> list_drivers() const {
        std::vector<std::string> names;
        for (auto& [name, _] : m_factories) names.push_back(name);
        return names;
    }

private:
    DriverRegistry() = default;
    std::unordered_map<std::string, FactoryFn> m_factories;
};

// Auto-registration helper
struct DriverAutoRegistrar {
    DriverAutoRegistrar(const char* name, DriverRegistry::FactoryFn fn) {
        DriverRegistry::get_instance().register_driver(name, std::move(fn));
    }
};

} // namespace apx::drivers

#define APX_REGISTER_DRIVER(ClassName, name) \
    static apx::drivers::DriverAutoRegistrar _apx_driver_reg_##ClassName( \
        name, []() -> std::unique_ptr<apx::drivers::DriverBase> { \
            return std::make_unique<ClassName>(); \
        })
