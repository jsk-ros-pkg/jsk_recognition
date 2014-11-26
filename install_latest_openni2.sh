#!/bin/sh
set -e

# rm -rf /tmp/openni2
# mkdir -p /tmp/openni2
# cd /tmp/openni2

# git clone https://github.com/atuleu/debian-openni2.git
# cd debian-openni2
# git-buildpackage -uc -us
# cd ..
# sudo dpkg -i libopenni2_2.2.0.33~beta2-1~dev3_amd64.deb libopenni2-dbg_2.2.0.33~beta2-1~dev3_amd64.deb libopenni2-dev_2.2.0.33~beta2-1~dev3_amd64.deb

sudo sh -c 'cat <<EOF > /etc/udev/rules.d/40-libopenni2-0.rules
SUBSYSTEM=="usb", ATTR{idProduct}=="0609", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
EOF
'
sudo service udev restart

echo done, Please compile openni2_camera from source code
