cmake_minimum_required(VERSION 3.5)
project(crsf_radio_tools)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(send_crsf_channels_test
    test/test_send_crsf_channels.py
  )
  ament_add_pytest_test(joy_to_crsf_test
    test/test_joy_to_crsf.py
  )
endif()

install(PROGRAMS
  scripts/send_crsf_channels.py
  scripts/joy_to_crsf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
