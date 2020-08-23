cmake_minimum_required(VERSION 2.8.3)
project(teachbot_express)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(PROGRAMS
   bin/www.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

