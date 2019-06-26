/*!
 * @file    test_main.cpp
 * @author  Gabriel Hottiger
 * @date    Nov, 2017
 */

#include <gtest/gtest.h>

using ::testing::EmptyTestEventListener;
using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

/* RUN TESTS */
int main(int argc, char **argv){
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
