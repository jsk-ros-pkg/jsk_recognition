# Install PCL from source

The following settings can be applied at the same time by combining cmake options.

## Install PCL from source without SSE

This is sometimes needed for [AttentionClipper](https://github.com/jsk-ros-pkg/jsk_recognition/issues/2380).

```bash
cd
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar xvzf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPCL_ENABLE_SSE:BOOL=FALSE
make -j 4
sudo make install
```

And rebuild all PCL related packges from source

## Install PCL from source with CUDA

This is needed for Kinfu.

```bash
cd
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar xvzf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DWITH_CUDA:BOOL=ON
make -j 4
sudo make install
```

And rebuild all PCL related packges from source
