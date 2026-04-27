std_msgs/Header header

# Pose of the camera in header.frame_id.
geometry_msgs/Pose camera_pose

# Texture image for this keyframe. Width/height define the Delaunay bounds.
sensor_msgs/Image image

# Optional keyframe id from the upstream odometry/VIO/SLAM system.
uint64 frame_id

projective_mesher_msgs/LandmarkObservation[] observations
