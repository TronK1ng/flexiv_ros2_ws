# One detected marker, expressed in the configured base frame.

# AprilTag id.
uint16 id

# Marker centre pose. The z axis points out of the marker plane.
geometry_msgs/PoseStamped pose

# Physical edge length of the marker in m, for sanity checking.
geometry_msgs/Vector3 size

# Quality metrics: pixel area of the marker and the PnP reprojection error.
float64 area
float64 reprojection_error
