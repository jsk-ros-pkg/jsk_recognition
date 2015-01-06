#!/bin/sh
set -e

sudo apt-get install git-buildpackage libudev-dev default-jdk
if [ ! -e /tmp/openni2 ]; then
    mkdir -p /tmp/openni2
    cd /tmp/openni2
    git clone https://github.com/atuleu/debian-openni2.git
else
    cd /tmp/openni2/debian-openni2
    git clean -xfd
    git reset --hard
    cd ..
fi
echo "override_dh_shlibdeps:\n\tdh_shlibdeps \$@ -- --ignore-missing-info" >> /tmp/openni2/debian-openni2/debian/rules

cd debian-openni2
git-buildpackage -uc -us --git-ignore-new
cd ..
sudo dpkg -i libopenni2_2.2.0.33~beta2-1~dev3_amd64.deb
sudo dpkg -i libopenni2-dbg_2.2.0.33~beta2-1~dev3_amd64.deb libopenni2-dev_2.2.0.33~beta2-1~dev3_amd64.deb

sudo sh -c 'cat <<EOF > /etc/udev/rules.d/40-libopenni2-0.rules
SUBSYSTEM=="usb", ATTR{idProduct}=="0609", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
EOF
'
sudo service udev restart

echo done, Please compile openni2_camera from source code by
echo wstool set --git openni2_camera https://github.com/ros-drivers/openni2_camera.git -v ${ROS_DISTRO}-devel
