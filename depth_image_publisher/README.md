# depth_image_publisher

[![CI](https://github.com/knorth55/depth_image_publisher/actions/workflows/main.yml/badge.svg)](https://github.com/knorth55/depth_image_publisher/actions/workflows/main.yml)

Image publisher for depth image

## Sample 

### 16UC1

```
rosrun depth_image_publisher depth_image_publisher $(rospack find depth_image_publisher)/sample/sample_16uc1.png
```

### 32FC1

```
rosrun depth_image_publisher depth_image_publisher $(rospack find depth_image_publisher)/sample/sample_16uc1.png _encoding:=32FC1
```
