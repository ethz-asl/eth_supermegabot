#!/bin/bash

# Generate urdf and store it in a file
echo "Generating smb.urdf in $(rospack find smb_model)/resources ..."
rosrun xacro xacro -io $(rospack find smb_model)/resources/smb.urdf $(rospack find smb_description)/urdf/smb_standalone.urdf.xacro

echo "Done!"
