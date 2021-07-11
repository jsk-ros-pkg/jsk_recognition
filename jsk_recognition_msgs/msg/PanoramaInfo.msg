# This message defines meta information for a panorama image.

#######################################################################
#                     Image acquisition info                          #
#######################################################################
std_msgs/Header header

#######################################################################
#                      Image projection info                          #
#######################################################################
string projection_model

#######################################################################
#                     Field of View Parameters                         #
#######################################################################
float64 theta_min
float64 theta_max
float64 phi_min
float64 phi_max
