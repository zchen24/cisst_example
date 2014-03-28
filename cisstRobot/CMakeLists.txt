# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstRobot
                              )

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  add_executable (FuntionRn mainFunctionRn.cpp)
  
  # link against cisst libraries (and dependencies)
  cisst_target_link_libraries (FuntionRn ${REQUIRED_CISST_LIBRARIES})

  # link other libraries
  # target_link_libraries (sawRobotIO1394QtConsole ${sawRobotIO1394_LIBRARIES})

endif (cisst_FOUND_AS_REQUIRED)