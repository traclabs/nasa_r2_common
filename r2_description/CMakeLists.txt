cmake_minimum_required(VERSION 2.8.3)
project(r2_description)

find_package(catkin REQUIRED cmake_modules)
find_package(xacro REQUIRED)

set(urdf_files "")

# Xacro files
file(GLOB model_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/xacro/*/*.xacro)

# Robot files
file(GLOB urdf_xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.xacro)

# Create urdf dir
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/urdf)

foreach(it ${urdf_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  message(" processing urdf file :${basepath}/${basename}")

  set(expanded_file "${CMAKE_CURRENT_SOURCE_DIR}/urdf/${basename}.urdf")
  message("   produce expanded file :${expanded_file}")

  xacro_add_xacro_file(${it} ${expanded_file})
  
  set(urdf_files ${urdf_files} ${expanded_file})
endforeach(it)

add_custom_target(r2_description_media_files ALL DEPENDS ${urdf_files})

catkin_package(
  CFG_EXTRAS r2-extras.cmake
)

install(
  DIRECTORY
    robots
    urdf
    meshes
    materials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "deprecated" EXCLUDE
)
