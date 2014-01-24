Header header
uint8 MODEL_PLANE=0
uint8 MODEL_CYLINDER=0
uint8 model_type

geometry_msgs/PolygonStamped target_plane

geometry_msgs/PointStamped center
geometry_msgs/Vector3Stamped direction
float64 radius

# parameters, 0 means 
float64 max_distance
float64 eps_angle
