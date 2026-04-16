std_msgs/Header header
bool detected
bool pose_valid
int32 tag_id
string tag_family
float32 tag_size_m
float32 confidence
float32 reproj_err_px
# Tag pose in the camera optical frame named by camera_frame_id.
string camera_frame_id
geometry_msgs/Point position_camera_m
# Base-relative tag pose. In OV-integrated launch this base frame is the derived
# OV body frame (for example uav_ov_body), not necessarily uav_base_link.
string base_frame_id
bool position_base_valid
geometry_msgs/Point position_base_m
# Tag pose in the odometry/world frame carried by odom_frame_id.
string odom_frame_id
string odom_child_frame_id
bool position_odom_valid
geometry_msgs/Point position_odom_m
float32 odometry_sample_age_s
