cmake_minimum_required(VERSION 2.8.5)

# Detect if it is called by catkin
IF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING)
  SET(CATKIN_BUILD TRUE)
ENDIF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING)

MESSAGE(STATUS "CATKIN_BUILD=${CATKIN_BUILD}")
   
PROJECT( find_object_2d )

# Catkin doesn't support multiarch library path, 
# fix to "lib" if not set by user.
IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
   set(CMAKE_INSTALL_LIBDIR "lib")
ENDIF(NOT DEFINED CMAKE_INSTALL_LIBDIR)

INCLUDE(GNUInstallDirs)

#######################
# VERSION
#######################
SET(PROJECT_VERSION "0.6.1")
SET(PROJECT_PREFIX find_object)

STRING(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
LIST(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
LIST(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
LIST(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)

ADD_DEFINITIONS(-DPROJECT_PREFIX="${PROJECT_PREFIX}")
ADD_DEFINITIONS(-DPROJECT_VERSION="${PROJECT_VERSION}")
ADD_DEFINITIONS(-DPROJECT_NAME="${PROJECT_NAME}")

####### DEPENDENCIES #######
FIND_PACKAGE(OpenCV REQUIRED) # tested on 2.3.1

# look for Qt5 before Qt4
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Network QUIET)
IF(NOT Qt5_FOUND)
   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
ENDIF(NOT Qt5_FOUND)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS) # To avoid conflicts with boost signals used in ROS

SET(NONFREE 0)
IF(OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND)
   SET(NONFREE 1)
ENDIF(OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND)

CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_PREFIX}/Version.h)

   #ROS Catkin build

   ## Find catkin macros and libraries
   ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
   ## is used, also find other catkin packages
   find_package(catkin REQUIRED COMPONENTS 
           cv_bridge roscpp rospy sensor_msgs std_msgs image_transport genmsg message_filters tf
   )

   ## Generate messages in the 'msg' folder
   add_message_files(
      FILES
      ObjectsStamped.msg
   )

   ## Generate added messages and services with any dependencies listed here
   generate_messages(
      DEPENDENCIES
      std_msgs
      sensor_msgs
   )

   ###################################
   ## catkin specific configuration ##
   ###################################
   ## The catkin_package macro generates cmake config files for your package
   ## Declare things to be passed to dependent projects
   ## INCLUDE_DIRS: uncomment this if you package contains header files
   ## LIBRARIES: libraries you create in this project that dependent projects also need
   ## CATKIN_DEPENDS: catkin_packages dependent projects also need
   ## DEPENDS: system dependencies of this project that dependent projects also need
   catkin_package(
     CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf
   )

   ###########
   ## Build ##
   ###########
   ADD_SUBDIRECTORY( src )

   #############
   ## Install ##
   #############
   ## Mark other files for installation (e.g. launch and bag files, etc.)
   install(FILES
      launch/find_object_2d_gui.launch
      launch/find_object_2d.launch
      launch/find_object_3d.launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   )

