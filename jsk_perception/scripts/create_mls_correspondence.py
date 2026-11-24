#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import scipy.io


def main():
    home_dir = os.environ['HOME']
    with open(os.path.join(home_dir, '.ros/l_img_crop.json'), 'r') as f:
        l_json_load = json.load(f)
    with open(os.path.join(home_dir, '.ros/r_img_crop.json'), 'r') as f:
        r_json_load = json.load(f)
    fixedPoints = r_json_load['shapes'][0]['points']
    movingPoints = l_json_load['shapes'][0]['points']
    if len(fixedPoints) != len(movingPoints):
        print('[error] The number of points in both images is different.')
        exit()
    dic = {'fixedPoints': fixedPoints,
           'movingPoints': movingPoints}
    mat_path = os.path.join(home_dir, '.ros/Correspondence_points_for_MLS.mat')
    scipy.io.savemat(mat_path, dic)
    print('{} point pairs are saved to {}'.format(len(fixedPoints), mat_path))


if __name__ == '__main__':
    main()
