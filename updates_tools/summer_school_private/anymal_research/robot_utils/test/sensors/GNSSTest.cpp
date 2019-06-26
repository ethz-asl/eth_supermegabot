/*!
* @file     GNSSTest.cpp
* @author   Dominic Jud
* @date     June 12, 2017
* @brief
*/

#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

#include "robot_utils/sensors/GNSS.hpp"

TEST(GNSSTest, besselEllipsoidToMercator)
{
    using namespace robot_utils;

    //Example Rigi: https://www.swisstopo.admin.ch/content/swisstopo-internet/de/topics/survey/reference-systems/switzerland/_jcr_content/contentPar/tabs/items/dokumente_publikatio/tabPar/downloadlist/downloadItems/517_1459343190376.download/refsys_d.pdf
    Eigen::Vector3d src = Eigen::Vector3d(0.821317799,0.148115967,500.0);
    Eigen::Vector3d dst = Eigen::Vector3d(79520.05,12273.44,500.0);
    Eigen::Vector3d calcDest;

    GNSS gnss = GNSS();
    calcDest = gnss.besselEllipsoidToMercator(src(0),src(1),src(2));

    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest, dst, 0.01, "ellipsodToMercator");
}
