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
    fixedPoints = l_json_load['shapes'][0]['points']
    movingPoints = r_json_load['shapes'][0]['points']
    dic = {'fixedPoints': fixedPoints,
           'movingPoints': movingPoints}
    scipy.io.savemat(
        os.path.join(home_dir, '.ros/Correspondence_points_for_MLS.mat'), dic)


if __name__ == '__main__':
    main()
