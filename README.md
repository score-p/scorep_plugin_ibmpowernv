# IBM PowerNV Score-P Plugin
This plugin tracks the power consumption of a Power 9-system.

## Usage
### Environment Variables
- `SCOREP_METRIC_PLUGINS=ibmpowernv_plugin`
- `SCOREP_METRIC_IBMPOWERNV_PLUGIN` (required)
    `*` (for all) OR comma-separated list of metrics to collect. See [here](src/occ_sensor_t.cpp) for available metrics.
- `SCOREP_METRIC_IBMPOWERNV_PLUGIN_INTERVAL` (optional, default: 10ms, case sensitive)
    Interval of testing.
    The value is a duration string which will be parsed, e.g. `100ms`.
    
### Testing
For testing grab [a sample occ inband sensors file](https://github.com/score-p/scorep_plugin_ibmpowernv/wiki/occ_inband_sensors_20210301T090454Z) and set the environment variable `SCOREP_METRIC_IBMPOWERNV_PLUGIN_OCC_FILENAME` to its location.
Provided your built `libibmpowernv_plugin.so` is in the current directory and you have [lo2s](https://github.com/tud-zih-energy/lo2s) installed, use this to generate a sample test trace:

```
LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH SCOREP_METRIC_PLUGINS=ibmpowernv_plugin SCOREP_METRIC_IBMPOWERNV_PLUGIN='*' SCOREP_METRIC_IBMPOWERNV_PLUGIN_OCC_FILENAME=./occ_inband_sensors_20210301T090454Z SCOREP_METRIC_IBMPOWERNV_PLUGIN_INTERVAL="0.1s"  lo2s -v -- sleep 1
```

(expected: `occ_power_system` always reads 471 W, `occ_power_system_acc` always 5415690.465 J)
    
## Acknowledgements
Copied structure of the source from [the meminfo Score-P plugin](https://github.com/score-p/scorep_plugin_meminfo).

Copied and adjusted code to read the sensors from [Variorum](https://github.com/llnl/variorum), which in turn is originally from [here](https://github.com/shilpasri/inband_sensors/blob/master/p9_inband_sensors.c).

Used function for time parsing from [metricq](https://github.com/metricq/metricq-cpp/blob/30bfccd61cff163885c4625c1fe810255ca95a11/src/chrono.cpp#L84-L130).
