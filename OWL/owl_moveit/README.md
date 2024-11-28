# OWL Moveit #

This repository contain a simple ROS1 package  `owl_moveit_driver` which can allow to use moveit with OWL Robot. It usese `owl_robot_client` to send a JointTrajectory message to OWL controller sampled at 250 Hz at max velocity limit of 50 degrees/sec.