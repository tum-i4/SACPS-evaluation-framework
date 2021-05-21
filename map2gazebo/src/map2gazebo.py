#!/usr/bin/env python


__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "10/2020"

"""
Original source: https://github.com/shilohc/map2gazebo by Shiloh Curtis

-> Modified by Sebastian Bergemann to fit to our use case and framework/wrapper.

Description:
This node will take the occupancy grid published by map_server from the current map image.
It will create then a mesh based on the occupancy state of each cell and export it to the
models folder, from where it then will be included by a generic gazebo world file, which
can in the end be opened/taken by the gazebo simulator.

This enables us to have always a gazebo representation of our current map, even if it is
compeltely custom and randomly generated each time.
"""

# IMPORTS
# ---------------------------------------------

import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation

import rospy
import rospkg

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "world_generator"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables
wrapper_namespace = ""
export_path = ""

# CODE
# ---------------------------------------------


class MapConverter(object):
    def __init__(self, map_topic, threshold=1, height=2.0):
        # self.test_map_pub = rospy.Publisher(
        #     "test_map", OccupancyGrid, latch=True, queue_size=1)
        # rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        self.map_topic = map_topic
        self.threshold = threshold
        self.height = height
        # Probably there's some way to get trimesh logs to point to ROS
        # logs, but I don't know it.  Uncomment the below if something
        # goes wrong with trimesh to get the logs to print to stdout.
        # trimesh.util.attach_to_log()

    # def map_callback(self, map_msg):
    #     rospy.loginfo("Received map")
    #     map_dims = (map_msg.info.height, map_msg.info.width)
    #     map_array = np.array(map_msg.data).reshape(map_dims)

    #     # set all -1 (unknown) values to 0 (unoccupied)
    #     map_array[map_array < 0] = 0
    #     contours = self.get_occupied_regions(map_array)
    #     meshes = [self.contour_to_mesh(c, map_msg.info) for c in contours]

    #     corners = list(np.vstack(contours))
    #     corners = [c[0] for c in corners]
    #     # self.publish_test_map(corners, map_msg.info, map_msg.header)
    #     mesh = trimesh.util.concatenate(meshes)

    #     # Export as STL or DAE
    #     mesh_type = rospy.get_param(
    #         "/" + wrapper_namespace + "/world_gen_mesh_type")
    #     if mesh_type == "stl":
    #         with open(export_path + "/map.stl", 'w') as f:
    #             mesh.export(f, "stl")
    #         rospy.loginfo("Exported STL.  You can shut down this node now")
    #     elif mesh_type == "dae":
    #         with open(export_path + "/map.dae", 'w') as f:
    #             f.write(trimesh.exchange.dae.export_collada(mesh))
    #         rospy.loginfo("Exported DAE.  You can shut down this node now")

    def convert_only_once(self):
        # get base map (provided by map_server directly from the provided map image)
        base_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t'" +
                      rospy.get_caller_id() + "' has received the map.\n")

        map_dims = (base_map.info.height, base_map.info.width)
        map_array = np.array(base_map.data).reshape(map_dims)

        # set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array)
        meshes = [self.contour_to_mesh(c, base_map.info) for c in contours]

        corners = list(np.vstack(contours))
        corners = [c[0] for c in corners]
        # self.publish_test_map(corners, base_map.info, base_map.header)
        mesh = trimesh.util.concatenate(meshes)

        # Export as STL or DAE
        mesh_type = rospy.get_param(
            "/" + wrapper_namespace + "/world_gen_mesh_type")
        if mesh_type == "stl":
            with open(export_path + "/map.stl", 'w') as f:
                mesh.export(f, "stl")
            rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t'" + rospy.get_caller_id() +
                          "' has created and exported the new mesh as STL.\n")
        elif mesh_type == "dae":
            with open(export_path + "/map.dae", 'w') as f:
                f.write(trimesh.exchange.dae.export_collada(mesh))
            rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t'" + rospy.get_caller_id() +
                          "' has created exported the new mesh as DAE.\n")

    # def publish_test_map(self, points, metadata, map_header):
    #     """
    #     For testing purposes, publishes a map highlighting certain points.
    #     points is a list of tuples (x, y) in the map's coordinate system.
    #     """
    #     test_map = np.zeros((metadata.height, metadata.width))
    #     for x, y in points:
    #         test_map[y, x] = 100
    #     test_map_msg = OccupancyGrid()
    #     test_map_msg.header = map_header
    #     test_map_msg.header.stamp = rospy.Time.now()
    #     test_map_msg.info = metadata
    #     test_map_msg.data = list(np.ravel(test_map))
    #     self.test_map_pub.publish(test_map_msg)

    def get_occupied_regions(self, map_array):
        """
        Get occupied regions of map
        """
        map_array = map_array.astype(np.uint8)
        _, thresh_map = cv2.threshold(
            map_array, self.threshold, 100, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
            thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)[-2:]
        # Using cv2.RETR_CCOMP classifies external contours at top level of
        # hierarchy and interior contours at second level.
        # If the whole space is enclosed by walls RETR_EXTERNAL will exclude
        # all interior obstacles e.g. furniture.
        # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html

        # # just for testing:
        # print(contours)
        # print(len(contours))
        # print(hierarchy)
        # img = np.ones((10, 10, 3), np.uint8) * 255
        # # black, red, green, blue, yellow
        # colors = [(0, 0, 0), (255, 17, 0), (48, 181, 0),
        #           (4, 0, 255), (277, 215, 43)]
        # for i in range(len(contours)):
        #     cv2.drawContours(img, contours[i], -1, colors[i], 1)
        # cv2.imshow('image', img)
        # cv2.waitKey(3000)

        original_code = False
        if original_code:
            # My problem with the original code:
            # The maps are always surrounded by a wall and every inner obstacle
            # which connects to them, will not be added to the final mesh
            # (when filtering for -1)
            hierarchy = hierarchy[0]
            corner_idxs = [i for i in range(
                len(contours)) if hierarchy[i][3] == -1]
            return [contours[i] for i in corner_idxs]
        else:
            return contours

    def contour_to_mesh(self, contour, metadata):
        height = np.array([0, 0, self.height])
        # s3 = 3**0.5 / 3.
        meshes = []
        for point in contour:
            x, y = point[0]
            vertices = []
            new_vertices = [
                coords_to_loc((x, y), metadata),
                coords_to_loc((x, y + 1), metadata),
                coords_to_loc((x + 1, y), metadata),
                coords_to_loc((x + 1, y + 1), metadata)]
            vertices.extend(new_vertices)
            vertices.extend([v + height for v in new_vertices])
            faces = [[0, 2, 4],
                     [4, 2, 6],
                     [1, 2, 0],
                     [3, 2, 1],
                     [5, 0, 4],
                     [1, 0, 5],
                     [3, 7, 2],
                     [7, 6, 2],
                     [7, 4, 6],
                     [5, 4, 7],
                     [1, 5, 3],
                     [7, 3, 5]]
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            if not mesh.is_volume:
                rospy.logdebug("Fixing mesh normals")
                mesh.fix_normals()
            meshes.append(mesh)
        mesh = trimesh.util.concatenate(meshes)
        mesh.remove_duplicate_faces()
        # mesh will still have internal faces.  Would be better to get
        # all duplicate faces and remove both of them, since duplicate faces
        # are guaranteed to be internal faces
        return mesh


def coords_to_loc(coords, metadata):
    x, y = coords
    loc_x = x * metadata.resolution + metadata.origin.position.x
    loc_y = y * metadata.resolution + metadata.origin.position.y
    # TODO: transform (x*res, y*res, 0.0) by Pose map_metadata.origin
    # instead of assuming origin is at z=0 with no rotation wrt map frame
    return np.array([loc_x, loc_y, 0.0])


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)

    wrapper_namespace = rospy.get_param("/wrapper_namespace")

    # Get all needed parameters
    map_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/base_map_topic")
    box_height = rospy.get_param(
        "/" + wrapper_namespace + "/world_gen_box_height")
    occupied_thresh = rospy.get_param(
        "/" + wrapper_namespace + "/world_gen_occupied_thresh")
    world_gen_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/gazebo_world_pkg")
    export_dir = rospy.get_param(
        "/" + wrapper_namespace + "/world_gen_export_dir")
    world_generator_status_topic = rospy.get_param(
        "/" + wrapper_namespace + "/world_generator_status_topic")

    status_pub = rospy.Publisher(
        world_generator_status_topic, String, queue_size=100)

    # get path to the launch file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(world_gen_pkg)
    export_path = pkg_path + export_dir

    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() + "' is ready - "
                  + "\n\t...will listen once to '" + map_topic + "' for the base map"
                  + "\n\t...will publish its status to '" + status_pub.resolved_name + "'"
                  + "\n\t...will export the new mesh for the gazebo world file to:\n\t'" + export_path + "'\n")

    converter = MapConverter(map_topic,
                             threshold=occupied_thresh, height=box_height)

    converter.convert_only_once()

    current_time = rospy.get_time()
    end_of_signaling = current_time + 5  # send signals for 5 secs
    rate = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown() and rospy.get_time() <= end_of_signaling:
        # signal ready:
        status_pub.publish(String("ready"))
        rate.sleep()
    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t'" +
                  rospy.get_caller_id() + "' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')
