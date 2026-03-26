# CMake generated Testfile for 
# Source directory: /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_mode_supervisor
# Build directory: /home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_supervisor_logic "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor/test_results/uav_mode_supervisor/test_supervisor_logic.gtest.xml" "--package-name" "uav_mode_supervisor" "--output-file" "/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor/ament_cmake_gtest/test_supervisor_logic.txt" "--command" "/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor/test_supervisor_logic" "--gtest_output=xml:/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor/test_results/uav_mode_supervisor/test_supervisor_logic.gtest.xml")
set_tests_properties(test_supervisor_logic PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor/test_supervisor_logic" TIMEOUT "60" WORKING_DIRECTORY "/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/build_static_scan/uav_mode_supervisor" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_mode_supervisor/CMakeLists.txt;92;ament_add_gtest;/home/th1rt3en/doco/uav_hw/src/workspace/doco_uav/src/uav_mode_supervisor/CMakeLists.txt;0;")
subdirs("uav_mode_supervisor__py")
subdirs("gtest")
