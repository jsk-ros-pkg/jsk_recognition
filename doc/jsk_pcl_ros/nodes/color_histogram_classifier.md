ColorHistogramClassifier
==============

![color_hist_rviz](https://user-images.githubusercontent.com/1901008/27892667-fc4ddd9c-623b-11e7-9b7a-9349f3711790.png)

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

* `~output` (`jsk_recognition_msgs/ClassificationResult`)

    Class from color histogram array

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

## Collecting Reference Color Histogram

1. First, launch color histogram without classifier

    ``` bash
    roslaunch jsk_pcl_ros sample_color_histogram.launch use_classifier:=false
    ```

2. Put one object on a plane

    Once you put a object, you can see color histogram in `rqt_image_view`

    ![color_histogram_2](https://user-images.githubusercontent.com/1901008/27870629-4f58019c-61de-11e7-8018-8a4d4ccd7e8d.png)


3. Open another terminal and get a histogram

    Now you can get actual histogram data by `rostopic echo`.

    ```bash
    rostopic echo -n1 /color_histogram/color_hiogram/output/histograms/histogram[0]
    [0.22604790329933167, 0.026946106925606728, 0.01646706648170948, 0.009730539284646511, 0.010479042306542397, 0.024700598791241646, 0.08757484704256058, 0.13173653185367584, 0.07335329055786133, 0.040419161319732666, 0.0359281450510025, 0.2365269511938095, 0.08008982241153717, 0.0]
    ---
    ```

    Write this vector data into `yaml` file so that classifier nodelet can load the histogram as reference.
    
    ```yaml
    # labels.yaml
    label_names:
      - coffee
    histograms:
      coffee: [0.22604790329933167, 0.026946106925606728, 0.01646706648170948, 0.009730539284646511, 0.010479042306542397, 0.024700598791241646, 0.08757484704256058, 0.13173653185367584, 0.07335329055786133, 0.040419161319732666, 0.0359281450510025, 0.2365269511938095, 0.08008982241153717, 0.0]
    ```

4. Load reference histograms to classifier

    Now you can register reference histograms to classifier in launch file

    ```xml
    <node name="color_histogram_classifier"
          pkg="jsk_pcl_ros" type="color_histogram_classifier">
      <rosparam command="load" file="labels.yaml" />
    </node>
    ```

5. Run and get result

    You will be able to get classification result as `jsk_recognition_msgs/ClassificationResult`.

See `jsk_pcl_ros/sample/sample_color_histogram.launch` for detail.
