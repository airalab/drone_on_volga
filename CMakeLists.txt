cmake_minimum_required(VERSION 2.8.3)
project(drone_on_volga)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
      #scripts/node
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        #launch/start.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
