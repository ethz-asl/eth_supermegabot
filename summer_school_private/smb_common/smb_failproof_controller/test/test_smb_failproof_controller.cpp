//
// Created by johannes on 10.04.19.
//

#include <gtest/gtest.h>
#include <smb_failproof_controller/SmbFailproofController.hpp>
#include <rocoma_plugin/rocoma_plugin.hpp>
#include <smb_roco/RocoCommand.hpp>
#include <smb_roco/RocoState.hpp>


TEST(testFailproofController, instantiation)
{
rocoma_plugin::FailproofControllerPlugin<smb_failproof_controller::SmbFailproofController, smb_roco::RocoState, smb_roco::RocoCommand> plugin;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}