ament_add_gtest(easynav_nav2_controller_tests easynav_nav2_controller_tests.cpp)
target_include_directories(easynav_nav2_controller_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(easynav_nav2_controller_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(easynav_nav2_controller_tests ${dependencies})
