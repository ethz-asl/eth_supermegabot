#!/usr/bin/env python

# Author: Peter Fankhauser, pfankhauser@anybotics.com

import roslib
roslib.load_manifest('rviz_world_loader')

import rospy
from os.path import isfile
from rosparam import load_file
from visualization_msgs.msg import Marker, MarkerArray


def load_markers():

    publisher = rospy.Publisher('~marker', MarkerArray, queue_size=1, latch=True)
    markerArray = MarkerArray();
    file_path = rospy.get_param('~world_file')
    if not file_path:
        rospy.loginfo('World description file path is empty.')
        return
    if not isfile(file_path):
        rospy.logwarn('World description file with path "' + file_path + '" does not exists.')
        return

    parameters = load_file(file_path)
    if not parameters:
        return
    for model in parameters[0][0]['models']:
        marker = Marker();
        if 'box' in model:
            marker.type = Marker.CUBE;
            model = model['box']
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        elif 'cylinder' in model:
            marker.type = Marker.CYLINDER;
            model = model['cylinder']
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        elif 'mesh' in model:
            marker.type = Marker.MESH_RESOURCE;
            model = model['mesh']
            marker.mesh_resource = model['mesh_resource']
            marker.mesh_use_embedded_materials = True
        else:
            continue

        marker.ns = model['name']
        marker.header.frame_id = model['frame_id']
        # marker.id = i # TODO Required?
        marker.action = Marker.ADD
        marker.pose.position.x = model['position']['x']
        marker.pose.position.y = model['position']['y']
        marker.pose.position.z = model['position']['z']
        marker.pose.orientation.x = model['orientation']['x']
        marker.pose.orientation.y = model['orientation']['y']
        marker.pose.orientation.z = model['orientation']['z']
        marker.pose.orientation.w = model['orientation']['w']
        marker.scale.x = model['scale']['x']
        marker.scale.y = model['scale']['y']
        marker.scale.z = model['scale']['z']
        markerArray.markers.append(marker)
        rospy.loginfo("World `%s` is published as marker.", marker.ns)

    publisher.publish(markerArray)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('rviz_world_loader', anonymous=False)
    load_markers()
