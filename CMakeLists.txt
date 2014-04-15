cmake_minimum_required(VERSION 2.8.3)
project(capabilities)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs std_srvs)

catkin_python_setup()

add_message_files(FILES
    Capability.msg
    CapabilityEvent.msg
    CapabilitySpec.msg
    Remapping.msg
    RunningCapability.msg
)

add_service_files(FILES
    EstablishBond.srv
    GetCapabilitySpec.srv
    FreeCapability.srv
    GetCapabilitySpecs.srv
    GetInterfaces.srv
    GetNodeletManagerName.srv
    GetProviders.srv
    GetRemappings.srv
    GetRunningCapabilities.srv
    GetSemanticInterfaces.srv
    StartCapability.srv
    StopCapability.srv
    UseCapability.srv
)

generate_messages(DEPENDENCIES std_msgs std_srvs)

catkin_package()

## Install scripts
install(PROGRAMS scripts/capability_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_nosetests(test)

    add_rostest(test/rostest/test_client/test_client_module.test)
    add_rostest(test/rostest/test_launch_manager/test_launch_manager.test)
    add_rostest(test/rostest/test_server/test_client.test)
    add_rostest(test/rostest/test_server/test_default_provider.test)
    add_rostest(test/rostest/test_server/test_dependent_capabilities.test)
    add_rostest(test/rostest/test_server/test_invalid_specs.test)
    add_rostest(test/rostest/test_server/test_package_white_black_lists.test)
    add_rostest(test/rostest/test_server/test_remapping.test)
    add_rostest(test/rostest/test_server/test_ros_services.test)
    add_rostest(test/rostest/test_server/test_white_black_lists.test)
    add_rostest(test/rostest/test_service_discovery/test_spec_index_from_service.test)
endif()
