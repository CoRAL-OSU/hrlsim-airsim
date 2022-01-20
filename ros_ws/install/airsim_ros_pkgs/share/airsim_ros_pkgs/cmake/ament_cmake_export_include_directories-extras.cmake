# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${airsim_ros_pkgs_DIR}/../../../include;${airsim_ros_pkgs_DIR}/../../../ /home/cthornton/data/local/college/research/arl/code/sims/airsim/AirSim2/external/rpclib/rpclib-2.3.0/include;/home/cthornton/data/local/college/research/arl/code/sims/airsim/AirSim2/AirLib/deps/eigen3;/home/cthornton/data/local/college/research/arl/code/sims/airsim/AirSim2/AirLib/include;/home/cthornton/data/local/college/research/arl/code/sims/airsim/AirSim2/MavLinkCom/include;/home/cthornton/data/local/college/research/arl/code/sims/airsim/AirSim2/MavLinkCom/common_utils;/usr/include/opencv4")

# append include directories to airsim_ros_pkgs_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'airsim_ros_pkgs' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND airsim_ros_pkgs_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
