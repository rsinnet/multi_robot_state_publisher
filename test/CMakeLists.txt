cmake_minimum_required(VERSION 3.5)
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(
    test_one_link
    test_one_link.launch
    test_one_link.cpp
    )
  target_link_libraries(
    test_one_link
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

  add_rostest_gtest(
    test_two_links_fixed_joint
    test_two_links_fixed_joint.launch
    test_two_links_fixed_joint.cpp
    )
  target_link_libraries(
    test_two_links_fixed_joint
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

  add_rostest_gtest(
    test_two_links_moving_joint
    test_two_links_moving_joint.launch
    test_two_links_moving_joint.cpp
    )
  target_link_libraries(
    test_two_links_moving_joint
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

  # Download needed data file
  catkin_download_test_data(
    ${PROJECT_NAME}_joint_states_indexed_bag
    http://wiki.ros.org/robot_state_publisher/data?action=AttachFile&do=get&target=joint_states_indexed.bag
    DESTINATION
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    FILENAME
    joint_states_indexed.bag
    MD5
    793e0b566ebe4698265a936b92fa2bba
    )

  add_rostest_gtest(
    test_joint_states_bag
    test_joint_states_bag.launch
    test_joint_states_bag.cpp
    )
  target_link_libraries(
    test_joint_states_bag
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )
  add_dependencies(
    test_joint_states_bag
    ${PROJECT_NAME}_joint_states_indexed_bag
    )

  add_rostest_gtest(
    test_subclass
    test_subclass.launch
    test_subclass.cpp
    )
  target_link_libraries(
    test_subclass
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

  install(
    FILES one_link.urdf
          pr2.urdf
          two_links_fixed_joint.urdf
          two_links_moving_joint.urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    )

endif()
