# add_executable(imu_initializer_test imu_initializer_test.cc)
# target_link_libraries(imu_initializer_test ${catkin_LIBRARIES} msm_sdk)

# add_executable(lidar_imu_mapping_test lidar_imu_mapping_test.cc)
# target_link_libraries(lidar_imu_mapping_test ${catkin_LIBRARIES} msm_sdk)

# add_executable(lidar_imu_odometry_test lidar_imu_odometry_test.cc)
# target_link_libraries(lidar_imu_odometry_test ${catkin_LIBRARIES} msm_sdk)

add_executable(low_cost_mapping_test low_cost_mapping_test.cc)
target_link_libraries(low_cost_mapping_test ${catkin_LIBRARIES} msm_sdk)

add_executable(low_cost_location_test low_cost_location_test.cc)
target_link_libraries(low_cost_location_test ${catkin_LIBRARIES} msm_sdk)

add_executable(lidar_session_test lidar_session_test.cc)
target_link_libraries(lidar_session_test msm_core)


add_executable(cloud_icp_registration_test cloud_icp_registration_test.cc)
target_link_libraries(cloud_icp_registration_test msm_core
  ${catkin_LIBRARIES}  )

# add_executable(backend_process_test backend_process_test.cc)
# target_link_libraries(backend_process_test msm_sdk ${catkin_LIBRARIES}  )

# add_executable(ground_feature_enhancement_test ground_feature_enhancement_test.cc )
# target_link_libraries(ground_feature_enhancement_test msm_core)

# add_executable(trajectory_test trajectory_test.cc)
# target_link_libraries(trajectory_test msm_core)

# add_executable(lio_initializer_test lio_initializer_test.cc )
# target_link_libraries(lio_initializer_test msm_core)

add_executable(lidar_tag_detection_test lidar_tag_detection_test.cc )
target_link_libraries(lidar_tag_detection_test msm_core)

add_executable(cloud_2d_visualization_test cloud_2d_visualization_test.cc )
target_link_libraries(cloud_2d_visualization_test msm_core)

add_executable(tag_based_map_rotation_test tag_based_map_rotation_test.cc )
target_link_libraries(tag_based_map_rotation_test msm_core)

add_executable(map_dot_test map_dot_test.cc )
target_link_libraries(map_dot_test msm_core)

add_executable(scan_undistortion_test scan_undistortion_test.cc )
target_link_libraries(scan_undistortion_test msm_core)