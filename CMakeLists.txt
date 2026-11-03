cmake_minimum_required(VERSION 3.8)

project(factory_delivery_amr_description)

find_package(ament_cmake REQUIRED)


# ============================================================================
# INSTALL ROBOT DESCRIPTION RESOURCES
# ============================================================================

install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    worlds
    models
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================================
# INSTALL PYTHON EXECUTABLES
# ============================================================================

install(
  PROGRAMS
    scripts/scissor_visualizer.py
    scripts/lift_gazebo_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================================
# FINISH AMENT PACKAGE
# ============================================================================

ament_package()