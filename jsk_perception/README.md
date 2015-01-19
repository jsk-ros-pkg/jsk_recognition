# jsk_perception

## nodes and nodelets
### jsk\_perception/GridLabel
![](images/grid_label.jpg)

Generate labels of grid.

#### Subscribing Topic
* `~input` (`sensor_msgs/CameraInfo` or `sensor_msgs/Image`)

  Input is `sensor_msgs/CameraInfo` or `sensor_msgs/Image`.
  If `use_camera_info` is true, `sensor_msgs/CameraInfo` will be used.
  If `use_camera_info` is false, `sensor_msgs/Image` will be used.
#### Publishing Topic
* `~output` (`sensor_msgs/Image (CV_32SC1)`)

  Output labels as image. Encoding is `CV_32SC1`.
#### Parameters
* `~label_size` (Integer, default: `32`)

  label size
* `~use_camera_info` (Boolean, default: `false`)

  if this parameter is true, it uses `sensor_msgs/CameraInfo` for `~input`.

### jsk\_perception/ApplyMaskImage
![](images/apply_mask_image.png)

Apply mask image to original image and visualize it. It's a utlity to visualize mask image.

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Original image.
* `~input/mask` (`sensor_msgs/Image`)

  Mask image.
#### Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Masked image. The image is clipped by bounding box of mask image and filtered by the mask.
  The region not specified by mask image is filled by 0.
* `~output/mask` (`sensor_msgs/Image`)

  Clipped mask image. The image is clipped by bounding box of mask image.

### jsk\_perception/SLICSuperPixels
![](images/slic_super_pixels.png)

Compute super pixels based on SLIC based on "SLIC Superpixels Compared to State-of-the-art Superpixel Methods" (TPAMI 2012).
jsk\_perception use implementation of https://github.com/PSMM/SLIC-Superpixels, https://github.com/berak/SLIC-Superpixels and https://github.com/garaemon/SLIC-SuperPixels.

Output of this node is an image and each value means label index.

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image.
#### Publishing Topic

* `~output` (`sensor_msgs/Image`)

  Output image. The encoding of image is `CV_32SC1` and each element value means label index.
* `~debug` (`sensor_msgs/Image`)

  Debug image, each border of cluster drawn by red contour.
* `~debug/mean_color` (`sensor_msgs/Image`)

  Debug image, each cluster is drawn by mean color of the cluster.
* `~debug/center_grid` (`sensor_msgs/Image`)

  Debug image, Center of each cluster is plotted by red dot.
#### Parameters
* `~number_of_super_pixels` (Integer, default: `100`)

  The number of super pixels.
* `~weight` (Integer, default: `4`)

  Weight of metrics between color and pixel distance.

### jsk\_perception/LabDecomposer
Decompose BGR/RGB image into separate planes in [CIE-Lab color space](http://en.wikipedia.org/wiki/Lab_color_space).

#### Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image.
#### Publishing Topic
* `~output/l` (`sensor_msgs/Image`)
* `~output/a` (`sensor_msgs/Image`)
* `~output/b` (`sensor_msgs/Image`)
  L*, a and b separated planes. Each image has CV_8UC encoding.
