# image_restoration

ROS package to restorate/denoise low quality image.

## Usage
By following command, you can publish denoised(=restorated) image topic. Please set correct args for your image(video) topic configuration.
```
roslaunch image_restoration sample_image_restoration.launch input_topic:=/usb_cam/image_raw output_topic:=/camera/processed_image
```
If you want to use compressed image, run following command.
```
roslaunch image_restoration sample_image_restoration_compressed.launch input_topic:=/usb_cam/image_raw output_topic:=/camera/processed_image
```

## Description
This package is for denoising image. When you use image-based control and need more precise image, this package might be very useful.
This package uses [Real-ESRGAN](https://github.com/xinntao/Real-ESRGAN) (python3 package). If you want to know the algorithm, please see there.
The weight file for denoising is to be downloaded when you `catkin build image_restoration`.


## Directory Explanation
```
.
├── CMakeLists.txt
├── README.md
├── launch
│   ├── sample_image_restoration.launch           # example for restorate raw image
│   └── sample_image_restoration_compressed.launch# example for restorate compressed image
├── package.xml
├── requirements.txt               # describe python package dependency for catkin_virtualenv
├── scripts
│   └── image_restoration.        # main script ( image process )
└── weights
    └── RealESRGAN_x4plus.         # download when you run `catkin build image_restoration`
```
