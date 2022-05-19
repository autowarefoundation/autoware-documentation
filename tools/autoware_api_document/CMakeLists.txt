cmake_minimum_required(VERSION 3.8)
project(autoware_api_document)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)

install(
  PROGRAMS script/generate
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE resource)
