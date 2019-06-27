# ROCOMA RQT
*(**Ro**bot **Co**ntroller **Ma**nager)*

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/rocoma_rqt/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/rocoma_rqt/job/master/)

## Description
A plugin to control the [Robot Controller Manager (ROCOMA)](https://bitbucket.org/leggedrobotics/rocoma)
## Usage
Launch the plugin with rosrun, add it to a launch file or include it into a rqt perspective:

    rosrun rocoma_rqt rocoma_rqt OR
    <node name="controller_manager_gui" pkg="rocoma_rqt" type="rocoma_rqt" output="screen"/>

The namespace the controller manager lives in can be set with the parameter ```highlevel_controller_ns```.