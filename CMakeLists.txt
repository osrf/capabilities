cmake_minimum_required(VERSION 2.8.3)
project(capabilities)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs std_srvs)

catkin_python_setup()

add_message_files(FILES
    Capability.msg
    CapabilityEvent.msg
    CapabilitySpec.msg
    RunningCapability.msg
)

add_service_files(FILES
    GetCapabilitySpecs.srv
    GetInterfaces.srv
    GetProviders.srv
    GetSemanticInterfaces.srv
    GetRunningCapabilities.srv
    StartCapability.srv
    StopCapability.srv
)

generate_messages(DEPENDENCIES std_msgs std_srvs)

catkin_package()

## Install scripts
install(PROGRAMS scripts/capability_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_add_nosetests(test)
