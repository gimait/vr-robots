search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=iCub.srdf
robot_name_in_srdf=iCub
moveit_config_pkg=iCub_moveit_config
robot_name=iCub
planning_group_name=RightArm
ikfast_plugin_pkg=iCub_RightArm_ikfast_plugin
base_link_name=chest
eef_link_name=r_gripper
ikfast_output_path=/home/gimait/workspace/vr_robots/src/iCub_RightArm_ikfast_plugin/src/iCub_RightArm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
