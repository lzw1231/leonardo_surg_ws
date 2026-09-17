// mtm_controllers/common/state_interface_utils.hpp
#pragma once

#include <limits>
#include <string>
#include <unordered_map>

namespace mtm_controllers{
    using StateInterfaceValueMap = std::unordered_map<std::string, std::unordered_map<std::string, double>>;

    inline constexpr double kUninitializedValue = std::numeric_limits<double>::quiet_NaN();

    inline double lookup_state_interface_value(
        const StateInterfaceValueMap& map,
        const std::string& name,
        const std::string& interface_name) {
        auto name_it = map.find(name);
        if (name_it == map.end()) {
            return kUninitializedValue;
        }
        auto if_it = name_it->second.find(interface_name);
        if (if_it == name_it->second.end()) {
            return kUninitializedValue;
        }
        return if_it->second;
    }
} // namespace mtm_controllers
