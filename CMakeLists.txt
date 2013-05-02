cmake_minimum_required(VERSION 2.8.3)
project(capabilities)

find_package(catkin REQUIRED COMPONENTS message_generation)

catkin_python_setup()

add_service_files(FILES
                  GetInterfaces.srv
                  GetProviders.srv
                  GetSemanticInterfaces.srv)

generate_messages()

catkin_package()

catkin_add_nosetests(test)
