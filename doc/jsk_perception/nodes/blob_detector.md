# BlobDetector
![](images/blob_detector.png)

Detect blob of binary image and output label of it.
The implementation is based on [Imura's implementation](http://oshiro.bpe.es.osaka-u.ac.jp/people/staff/imura/products/labeling).

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image. It should be single channel.
## Publishing Topic
* `~output` (`sensor_msgs/Image`)

  Label image. Each pixel value means the label which the pixel should belong to and 0 means
  the pixel is masked (black in `~input` image)

## Parameter
* `~min_size` (Integer, default: `10`)

  Minimum size of blob

## Sample

```bash
roslaunch jsk_perception sample_blob_detector.launch
```
