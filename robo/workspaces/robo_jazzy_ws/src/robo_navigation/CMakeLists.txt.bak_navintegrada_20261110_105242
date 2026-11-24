cmake_minimum_required(VERSION 3.8)
project(robo_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lateral_ws test/test_lateral_ws.py)
  ament_add_pytest_test(face_proxima test/test_face_proxima.py)
  ament_add_pytest_test(giro_frontal test/test_giro_frontal.py)
  ament_add_pytest_test(giro_integrado test/test_integrado_t.py TIMEOUT 120)
endif()

ament_package()
