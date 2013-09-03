# For all goal types.
int32 number
string description
int32 type

# For pick goals and place goals.
int32 object_id

# For pick goals.
geometry_msgs/Pose grasp_pose
geometry_msgs/Pose initial_object_pose
float32 grasp_distance
float32 gripper_joint_position

# For place goals.
bool ignore_yaw
geometry_msgs/Pose place_pose