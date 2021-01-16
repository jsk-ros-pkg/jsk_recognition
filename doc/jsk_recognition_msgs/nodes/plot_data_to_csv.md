# plot_data_to_csv.py

## What is this?

Subscribe `jsk_recognition_msgs/PlotData` and write data to file.


## Subscribing Topic

* `~input` (`jsk_recognition_msgs/PlotData`)

  Input plot data.


## Publishing Topic

None.


## Parameters

* `~filename` (String, default: `output_%04d.csv`)

  Path to output file.

  Integer place holder like `%04d` can be used to separate file.
  If the path does not contain place holder, this node will overwrite the file on every callback.


## Sample

```bash
roslaunch jsk_recognition_msgs sample_plot_data_to_csv.launch
```
