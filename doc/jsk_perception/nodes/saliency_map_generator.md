# SaliencyMapGenerator
![](images/saliency_map.png)

This nodelet is used to compute the saliency of a image. The image can be composed to any feature space, however the current nodelet is takes only 3 channel RGB image and computes the saliency map. 

The original code is implemented by the authors of the paper [Human detection using a mobile platform and novel features derived from a visual saliency mechanism] (http://www.sciencedirect.com/science/article/pii/S0262885609001371). Please cite this paper should you use the code. However, there are minor changes to the original code including the MultiThreading support using OpenMP.


## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Currently only supports "BGR8" (3 channel RGB intensity image)

## Publishing Topic
* `~output` (`sensor_msgs/Image`)

## Parameters
* `num_thread` (Int, default: `2`)
Number of threads to used for execution

* `print_fps` (Bool, default: `true`)
Prints the frame rate to the frame

