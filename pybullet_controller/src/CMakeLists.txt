cmake_minimum_required(VERSION 2.6)
PROJECT(  pybullet_controller )

INCLUDE( /home/robocomp/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple CameraRGBDSimple JoystickAdapter KinovaArm KinovaArm )
