
// gtest
#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include <any_rbdl/rbdl.h>

int main (int argc, char *argv[])
{
	rbdl_check_api_version (ANY_RBDL_API_VERSION);

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();

}
