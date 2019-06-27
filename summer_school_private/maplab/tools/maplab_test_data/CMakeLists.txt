cmake_minimum_required (VERSION 2.8)
project(maplab_test_data)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

install(DIRECTORY csv_feeder_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY end_to_end_test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY frame_localizer_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY map_resources_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY maplab_common_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY openvdb_interface_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY simulation_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY stereo_dense_reconstruction_test_data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY test_maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
cs_install()
cs_export(CFG_EXTRAS maplab-test-data.cmake maplab-import-test-maps.cmake)
