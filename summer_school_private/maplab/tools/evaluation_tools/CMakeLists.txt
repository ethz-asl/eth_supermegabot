project(evaluation_tools)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11)

# This macro ensures modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

cs_add_library(${PROJECT_NAME} src/evaluation-data-collector.cc)

cs_add_executable(dummy_app_for_unit_test src/test/dummy-app-for-unit-test.cc)

#########
# TESTS #
#########
catkin_add_nosetests(test/test_job.py)
catkin_add_nosetests(test/test_evaluation.py)

##########
# EXPORT #
##########
cs_install()
cs_export()
