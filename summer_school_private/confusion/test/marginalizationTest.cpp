/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>
#include "confusion/utilities/utilities.h"

using namespace confusion;

TEST(reorderTest, ShouldPass) {
	std::vector<std::pair<int,int>> segmentPositionAndSize;
	segmentPositionAndSize.push_back(std::make_pair(8,1));
	segmentPositionAndSize.push_back(std::make_pair(1,2));
	segmentPositionAndSize.push_back(std::make_pair(4,3));

	Eigen::MatrixXd test(10,10);
	test.setRandom();
	test.diagonal() << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;

	Eigen::VectorXd bstar(10);
	bstar << 0,1,2,3,4,5,6,7,8,9;

	reorderNormalEquations(test, bstar, segmentPositionAndSize);

//	std::cout << "out: \n" << test << std::endl;
//	std::cout << "bstar: " << bstar.transpose() << std::endl;

	Eigen::VectorXd result(10);
	result << 8,1,2,4,5,6,0,3,7,9;
	for (int i=0; i<10; ++i) {
		EXPECT_EQ(bstar(i),result(i));
		EXPECT_LT(fabs(test(i,i)-result(i)),1e-10);
	}
}

TEST(simpleMargTest, ShouldPass) {
	Eigen::MatrixXd H(10,10); H.setZero();
	Eigen::VectorXd b(10);
	for (int i=0; i<10; ++i) {
		H(i,i) = i+1;
		b(i) = i+1;
	}
//	std::cout << "H:\n" << H << std::endl;
//	std::cout << "\n\n\n" << std::endl;

	Eigen::MatrixXd Wout;
	Eigen::VectorXd eout;
	Eigen::MatrixXd HstarOut;
	Eigen::VectorXd bstarOut;
	EXPECT_TRUE(schurMarginalize(H, b, 3, Wout, eout, &HstarOut, &bstarOut));

//	std::cout << "Wout:\n" << Wout << std::endl;
//	std::cout << "eout:\n" << eout.transpose() << std::endl;
//	std::cout << "HstarOut:\n" << HstarOut << std::endl;
//	std::cout << "bstarOut:\n" << bstarOut.transpose() << std::endl;

	EXPECT_EQ(Wout.rows(),7);
	EXPECT_EQ(Wout.cols(),7);
	EXPECT_EQ(eout.rows(),7);
	EXPECT_EQ(HstarOut.rows(),7);
	EXPECT_EQ(HstarOut.cols(),7);
	EXPECT_EQ(bstarOut.rows(),7);

	for (int i=0; i<7; ++i) {
		EXPECT_EQ(HstarOut(i,i),i+4);
		EXPECT_EQ(bstarOut(i),i+4);
	}
}


int main(int argc, char** argv) {
	srand(time(NULL));

	std::cout.precision(10);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
