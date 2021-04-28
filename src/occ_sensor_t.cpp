// Copyright (C) 2021 Technische Universität Dresden, Germany
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// (1) Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// (2) Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in
// the documentation and/or other materials provided with the
// distribution.
//
// (3)The name of the author may not be used to
// endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <occ_sensor_t.hpp>

#include <scorep/SCOREP_MetricTypes.h>

#include <map>
#include <string>
#include <stdexcept>

/// helper class to allow setting all attributes of scorep::plugin::metric_property from constructor
class metric_type_constructable : public scorep::plugin::metric_property {
public:
    metric_type_constructable(std::string name,
                              std::string description,
                              std::string unit,
                              SCOREP_MetricMode _mode = SCOREP_METRIC_MODE_ABSOLUTE_POINT,
                              SCOREP_MetricValueType _type = SCOREP_METRIC_VALUE_DOUBLE,
                              SCOREP_MetricBase _base = SCOREP_METRIC_BASE_DECIMAL,
                              int64_t _exponent = 0) : scorep::plugin::metric_property(name, description, unit) {
        // note: set here, because parent class' attributes can't be set from initializer lists
        mode = _mode;
        type = _type;
        base = _base;
        exponent = _exponent;
    }
};

const std::map<occ_sensor_t, scorep::plugin::metric_property> occ_sensor_t::metric_properties_by_sensor = {
    // structure:
    // OCC-string identifier, bool whether to use accumulator or not
    // name for metric in trace, description, unit
    {{"PWRSYS", occ_sensor_sample_type::sample},
     metric_type_constructable("occ_power_system", "power intake of the entire system", "W", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_DOUBLE)},
    {{"PWRSYS", occ_sensor_sample_type::acc},
     metric_type_constructable(
         "occ_power_system_acc",
         "accumulator of consumed energy by entire system",
         "J", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
    {{"PWRSYS", occ_sensor_sample_type::timestamp},
     metric_type_constructable(
         "occ_power_system_timestamp",
         "timestamp for occ_power_system sensors",
         "?", SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_UINT64)},
};

SCOREP_MetricValueType occ_sensor_t::get_scorep_type() const {
    for (const auto it : occ_sensor_t::metric_properties_by_sensor) {
        if (*this == it.first) {
            return it.second.type;
        }
    }

    throw std::runtime_error("can't identify datatype for sensor " + name + " with type " + std::to_string(type));
}

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs)
{
    if (lhs.name != rhs.name) {
        return lhs.name < rhs.name;
    }

    return lhs.type < rhs.type;
}

bool operator==(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    return lhs.name == rhs.name && lhs.type == rhs.type;
}
