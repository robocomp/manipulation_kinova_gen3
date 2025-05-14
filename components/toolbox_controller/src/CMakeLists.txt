cmake_minimum_required(VERSION 2.6)
PROJECT(  toolbox_controller )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior GenericBase IMU OmniRobot RoboticsToolboxController )
