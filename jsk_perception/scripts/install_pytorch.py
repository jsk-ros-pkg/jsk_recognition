#!/usr/bin/env python

"""Install script of PyTorch. See http://pytorch.org for detail."""

import os
import platform
import shlex
import subprocess
import sys


def install_packages(pkgs):
    if not os.environ.get('CMAKE_PREFIX_PATH'):
        print('[ERROR] CMAKE_PREFIX_PATH is not set')
        sys.exit(1)

    prefix = os.environ['CMAKE_PREFIX_PATH'].split(':')[0]
    target = os.path.join(prefix, 'lib/python2.7/dist-packages')

    for pkg in pkgs:
        cmd = 'pip install -q --target %s %s' % (target, pkg)
        print('+ %s' % cmd)
        subprocess.call(shlex.split(cmd))


def main():
    pkgs = []

    # PyTorch
    if platform.platform().split('-')[0] == 'Linux':
        if os.path.exists('/usr/local/cuda-7.5/include/cuda.h'):
            pkgs.append('http://download.pytorch.org/whl/cu75/torch-0.1.10.post2-cp27-none-linux_x86_64.whl')  # NOQA
        elif os.path.exists('/usr/local/cuda-8.0/include/cuda.h'):
            pkgs.append('http://download.pytorch.org/whl/cu80/torch-0.1.10.post2-cp27-none-linux_x86_64.whl')  # NOQA
        else:
            pkgs.append('http://download.pytorch.org/whl/cu75/torch-0.1.10.post2-cp27-none-linux_x86_64.whl')  # NOQA

    install_packages(pkgs)


if __name__ == '__main__':
    main()
