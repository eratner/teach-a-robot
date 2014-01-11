# For all goal types.
int32 number
string description
int32 type

# For pick goals and place goals.
int32 object_id

# For pick goals.
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose initial_object_pose
float32 gripper_joint_position
float32 camera_phi
float32 camera_theta
float32 camera_radius

# For place goals.
bool ignore_yaw
geometry_msgs/Pose place_pose