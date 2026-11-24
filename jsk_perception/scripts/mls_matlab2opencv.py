#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python version of https://gist.github.com/dangkhoasdc/991b6913a43a2a726743

import os
import scipy.io


def main():
    home_dir = os.environ['HOME']
    mls = scipy.io.loadmat(os.path.join(home_dir, '.ros/XY_MLS_Grid_example2.mat'))
    yml_path = os.path.join(home_dir, '.ros/fisheye_stitcher_grid_xd_yd_3840x1920.yml')
    with open(yml_path, 'w') as f:
        f.write('%YAML:1.0\n')
        for label in ['Xd', 'Yd']:
            rows = mls[label].shape[0]
            cols = mls[label].shape[1]
            f.write('    {}: !!opencv-matrix\n'.format(label))
            f.write('        rows: {}\n'.format(rows))
            f.write('        cols: {}\n'.format(cols))
            f.write('        dt: f\n')
            f.write('        data: [ ')
            for i in range(mls[label].size):
                f.write(str(mls[label][i / cols][i % cols]))
                if i < mls[label].size - 1:
                    f.write(', ')
                    if (i % 4) == 3:
                        f.write('\n            ')
                else:
                    f.write(' ]\n')
    print('{} is exported.'.format(yml_path))


if __name__ == '__main__':
    main()
