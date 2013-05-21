Header header
# range sensor
sensor_msgs/Image intensity
sensor_msgs/Image confidence
sensor_msgs/Image depth
sensor_msgs/Image depth16
# sensor_msgs/PointCloud point_cloud_raw
sensor_msgs/CameraInfo range_info

# camera
sensor_msgs/Image left_image
sensor_msgs/Image right_image
sensor_msgs/CameraInfo left_info
sensor_msgs/CameraInfo right_info

# result
cr_capture/PixelIndices pixel_indices
sensor_msgs/PointCloud2 point_cloud
