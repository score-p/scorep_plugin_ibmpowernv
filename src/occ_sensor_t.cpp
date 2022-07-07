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

#define OCC_PLUGIN_ADD_SENSOR_DEFAULT(occ_str, scorep_str, desc, unit) \
    {{occ_str, occ_sensor_sample_type::sample, 0},\
     metric_type_constructable(scorep_str, desc, unit, SCOREP_METRIC_MODE_ABSOLUTE_POINT, SCOREP_METRIC_VALUE_DOUBLE)},\
    {{occ_str, occ_sensor_sample_type::acc_derivative, 0, "J"},\
     metric_type_constructable(scorep_str "_energy", desc " (energy)", "J", SCOREP_METRIC_MODE_ACCUMULATED_LAST, SCOREP_METRIC_VALUE_DOUBLE)},\
    {{occ_str, occ_sensor_sample_type::acc_derivative, 0},\
     metric_type_constructable(scorep_str "_from_energy", desc " (derived from accumulator)", unit, SCOREP_METRIC_MODE_ABSOLUTE_LAST, SCOREP_METRIC_VALUE_DOUBLE)}

const std::map<occ_sensor_sample_type, std::string> name_by_occ_sensor_sample_type = {
    {sample, "sample"},
    {timestamp, "timestamp"},
    {update_tag, "update_tag"},
    {acc_derivative, "acc_derivative"},
};

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

bool operator<(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    if (lhs.sensor_type != rhs.sensor_type) {
        return lhs.sensor_type < rhs.sensor_type;
    }

    return lhs.occ_num < rhs.occ_num;
}

bool operator==(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    return lhs.sensor_type == rhs.sensor_type && lhs.occ_num == rhs.occ_num;
}

bool operator!=(const occ_sensor_t& lhs, const occ_sensor_t& rhs) {
    return !(lhs == rhs);
}

bool operator<(const occ_metric_t& lhs, const occ_metric_t& rhs) {
    if (lhs.sensor != rhs.sensor) {
        return lhs.sensor < rhs.sensor;
    }

    return lhs.sample_type < rhs.sample_type;
}

bool operator==(const occ_metric_t& lhs, const occ_metric_t& rhs) {
    return lhs.sensor == rhs.sensor && lhs.sample_type == rhs.sample_type;
}
