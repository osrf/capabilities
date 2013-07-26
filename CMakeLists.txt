cmake_minimum_required(VERSION 2.8.3)
project(capabilities)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

catkin_python_setup()

add_message_files(FILES
    Capability.msg
    CapabilityEvent.msg
    RunningCapability.msg
)

add_service_files(FILES
    GetInterfaces.srv
    GetProviders.srv
    GetSemanticInterfaces.srv
    GetRunningCapabilities.srv
    StartCapability.srv
    StopCapability.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

catkin_add_nosetests(test)
