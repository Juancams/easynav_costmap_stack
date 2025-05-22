ament_add_gtest(costmap_mapsmanager_tests costmap_mapsmanager_tests.cpp)
target_include_directories(costmap_mapsmanager_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(costmap_mapsmanager_tests
  ${PROJECT_NAME}
)
ament_target_dependencies(costmap_mapsmanager_tests ${dependencies})
