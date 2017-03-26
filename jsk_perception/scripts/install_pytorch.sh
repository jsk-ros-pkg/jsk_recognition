#!/bin/bash

# Install script of PyTorch. See http://pytorch.org for detail.

set -x

if [ -e /usr/local/cuda-7.5/include/cuda.h ]; then
  pip install -q http://download.pytorch.org/whl/cu75/torch-0.1.10.post2-cp27-none-linux_x86_64.whl
elif [ -e /usr/local/cuda-8.0/include/cuda.h ]; then
  pip install -q http://download.pytorch.org/whl/cu80/torch-0.1.10.post2-cp27-none-linux_x86_64.whl
else
  pip install -q http://download.pytorch.org/whl/cu75/torch-0.1.10.post2-cp27-none-linux_x86_64.whl
fi

set +x
