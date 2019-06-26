/*!
* @file     remove_first_if_test.cpp
* @author   Philipp Leemann
* @date     Mar, 2018
* @brief
*/
#include <gtest/gtest.h>
#include <algorithm>

#include "std_utils/remove_first_if.hpp"

TEST(remove_first_if, IntVector) {
  std::vector<int> myVec{1,2,3,4,5,6,7,8,9,0};
  myVec.erase(std_utils::remove_first_if(myVec.begin(), myVec.end(), [](const int a){ return (a == 6); }), myVec.end());
  {
    std::vector<int> tmp{1, 2, 3, 4, 5, 7, 8, 9, 0};
    EXPECT_EQ(tmp, myVec);
  }

  myVec.erase(std_utils::remove_first_if(myVec.begin(), myVec.end(), [](const int a){ return (a == 1); }), myVec.end());
  {
    std::vector<int> tmp{2, 3, 4, 5, 7, 8, 9, 0};
    EXPECT_EQ(tmp, myVec);
  }

  myVec.erase(std_utils::remove_first_if(myVec.begin(), myVec.end(), [](const int a){ return (a == 0); }), myVec.end());
  {
    std::vector<int> tmp{2, 3, 4, 5, 7, 8, 9};
    EXPECT_EQ(tmp, myVec);
  }
}


TEST(remove_first_if, StructVector) {
  struct myStruct {
    myStruct(const int a): a_(a) {}

    int a_{0};

    // operator for EXPECT_EQ
    bool operator==(const myStruct& other) const { return (a_ == other.a_); }
  };
  std::vector<myStruct> myVec{{1}, {2}, {3}};
  myVec.erase(std_utils::remove_first_if(myVec.begin(), myVec.end(), [](const myStruct& a){ return (a.a_ == 2); }), myVec.end());
  {
    std::vector<myStruct> tmp{{1}, {3}};
    EXPECT_EQ(tmp, myVec);
  }

  myVec.erase(std_utils::remove_first_if(myVec.begin(), myVec.end(), [](const myStruct& a){ return (a.a_ == 3); }), myVec.end());
  {
    std::vector<myStruct> tmp{{1}};
    EXPECT_EQ(tmp, myVec);
  }
}
