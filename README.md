# IBM PowerNV Score-P Plugin
This plugin tracks the power consumption of a Power 9-system.

## Usage
### Environment Variables
- `SCOREP_METRIC_PLUGINS=ibmpowernv_plugin`
- `SCOREP_METRIC_IBMPOWERNV_PLUGIN` (required)
    regex for metrics to be collected (TODO allow comma-separated list)
- `SCOREP_METRIC_IBMPOWERNV_PLUGIN_INTERVAL` (optional, default: 10ms)
    Interval of testing.
    The value is a duration string which will be parsed, e.g. `100ms`.
    
## Acknowledgements
Copied structure of the source from [the meminfo Score-P plugin](https://github.com/score-p/scorep_plugin_meminfo).

Copied and adjusted code to read the sensors from [here](https://github.com/shilpasri/inband_sensors/blob/master/p9_inband_sensors.c).
