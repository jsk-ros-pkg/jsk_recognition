#!/usr/bin/env python

# set up parameters that we care about
PACKAGE = 'jsk_pcl_ros'

try:
    import imp
    imp.find_module(PACKAGE)
    from dynamic_reconfigure.parameter_generator_catkin import *;
except:
    import roslib; roslib.load_manifest(PACKAGE)
    from dynamic_reconfigure.parameter_generator import *;
gen = ParameterGenerator ()

gen.add("use_range_likelihood", bool_t, 0, "", False)
gen.add("range_likelihood_local_min_z", double_t, 0, "", 0.0, 0.0, 1.0)
gen.add("range_likelihood_local_max_z", double_t, 0, "", 0.0, 0.0, 1.0)
gen.add("use_occlusion_likelihood", bool_t, 0, "", False)
gen.add("min_inliers", int_t, 0, "", 10, 0, 1000)
gen.add("outlier_distance", double_t, 0, "", 0.1, 0.0, 1.0)
