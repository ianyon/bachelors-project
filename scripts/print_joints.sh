a=(l_shoulder_pan_joint l_shoulder_lift_joint l_upper_arm_roll_joint l_elbow_flex_joint l_forearm_roll_joint l_wrist_flex_joint l_wrist_roll_joint r_shoulder_pan_joint r_shoulder_lift_joint r_upper_arm_roll_joint r_elbow_flex_joint r_forearm_roll_joint r_wrist_flex_joint r_wrist_roll_joint)

for e in "${a[@]}"
do 
	printf "%s " "$e"
	rosservice call gazebo/get_joint_properties $e | grep position
done