uint8 VALID=0
uint8 NO_LEFT_RECT=1
uint8 NO_RIGHT_RECT=2
uint8 NO_DEPTH=3
uint8 FAILED_ARUN=4

int64 id

# Pixel coordinates in the camera image. Only x/y are used.
geometry_msgs/Point keypoint

# Landmark position in the mesh/world frame named by MesherInput.header.frame_id.
geometry_msgs/Point world_point

# Optional landmark position in the current camera frame.
bool has_camera_point
geometry_msgs/Point camera_point

uint8 status
