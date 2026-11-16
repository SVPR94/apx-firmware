// SPDX-License-Identifier: CC-BY-NC-4.0
// Copyright (c) 2026 Savita Pawar <savitapawar0402@gmail.com>
// Author: Savita Pawar (SVPR94)
// Created: 2026
#pragma once
#include <unordered_map>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace apx {

class VehicleBase {
public:
    virtual ~VehicleBase() = default;
    virtual const char* name() const = 0;
    virtual void configure() = 0;
};

class VehicleRegistry {
public:
    using FactoryFn = std::function<std::unique_ptr<VehicleBase>()>;

    static VehicleRegistry& get_instance() {
        static VehicleRegistry inst;
        return inst;
    }

    void register_vehicle(const std::string& name, FactoryFn fn) {
        m_factories[name] = std::move(fn);
    }

    std::unique_ptr<VehicleBase> create(const std::string& name) const {
        auto it = m_factories.find(name);
        if (it == m_factories.end()) return nullptr;
        return it->second();
    }

    std::vector<std::string> list() const {
        std::vector<std::string> names;
        for (auto& [name, _] : m_factories) names.push_back(name);
        return names;
    }

private:
    VehicleRegistry() = default;
    std::unordered_map<std::string, FactoryFn> m_factories;
};

struct VehicleAutoRegistrar {
    VehicleAutoRegistrar(const char* name, VehicleRegistry::FactoryFn fn) {
        VehicleRegistry::get_instance().register_vehicle(name, std::move(fn));
    }
};

} // namespace apx

#define APX_REGISTER_VEHICLE(name, ClassName) \
    static apx::VehicleAutoRegistrar _apx_vehicle_reg_##ClassName( \
        name, []() -> std::unique_ptr<apx::VehicleBase> { \
            return std::make_unique<ClassName>(); \
        })
