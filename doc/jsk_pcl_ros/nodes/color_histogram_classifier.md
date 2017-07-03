ColorHistogramClassifier
==============

![detect_cans](https://user-images.githubusercontent.com/1901008/26961630-2ff9326c-4d1b-11e7-9a6e-2e149ae4ffca.png)

Classify point indices using color histogram by comparing with reference histogram array

Methods for histogram comparison is configurable from multiple methods. (See parameter `~compare_policy`)
After computing distance between input histograms and reference, classify by their labels (See parameter `~detection_threshold`)
Reference histograms are loaded from rosparam on start.

## Subscribing Topics

* `~input` (`jsk_recognition_msgs/ColorHistogram`)

    Input color histogram to be classified  

* `~input/array` (`jsk_recognition_msgs/ColorHistogramArray`)

    Input color histogram array to be classified

## Publishing Topics

* `~output` (`jsk_recognition_msgs/ColorHistogramArray`)

    Filtered color histogram array

* `~output/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Filtered cluster point indices

## Parameters

* `~queue_size` (`Int`, default: `100`)

    Queue size for message synchronization

* `~compare_policy` (`Enum[Int]`, default: `CORRELATION`)

    Policy for histogram values to compare

    - 0: `CORRELATION`
        - Use correlation
    - 1: `BHATTACHARYYA`
        - Use bhattacharyya distance
    - 2: `INTERSECTION`
        - Use vector intersection
    - 3: `CHISQUARE`
        - Use chi-square between two vectors
    - 4: `KL_DIVERGENCE`
        - Use Kullback-Leibler divergence for comparing two vectors

* `~label_names` (`String[]`, required)

    Reference class names

    This parameter is required on start

* `~histograms/<label name>` (`Double[]`, required)

    Reference histogram vector for each class

    Length of all histograms must be the same.

* `~detection_threshold` (`Double`, default: `0.8`)

    Color histograms and point cloud indices whose similarities are above this value are published as filtered topics.
