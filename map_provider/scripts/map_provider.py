#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "09/2020"

"""
TASK:
---------------------------------------------
Gets base map from the map_server directly from the provided map image.
Transform this into a map which is then used by all the other wrapper nodes and which should be used also by all the simulator nodes.
Currently the only transform action is to resize the initial map based on the wanted cell resolution by the user (no orientation, etc. is changed).

HELPFUL INFOMRATION:
---------------------------------------------
If you are familiar with the map layout provided by the official ROS map_server: this is exactly the same layout!

If "map" is the received object (OccupancyGrid taken from the PROVIDED_MAP_TOPIC):
The startpoint of the map (first element of the array) is the bottom left corner of the inital image of the map (identical with the layout of the map from map_server).
This bottom left corner (first array element) maps to the real world position given by map.info.origin.
From this origin / start point the x direction is horizontal (to the right) and the y direction is vertical (to the top).
Each cell has the size map.info.resolution x map.info.resolution (resolution is in m/cell).
The map is then a rectangle (or normally square) which has from the start point <map.info.width> cells in x direction and <map.info.height> in y direction.
If you want to have the real world distances in m: length/width = map.info.width*map.info.resolution and height = map.info.height*map.info.resolution

If you have a real world position (x,y) and want to get the cell containing this position:
  cell_x = min(int((x - map.info.origin.position.x) / map.info.resolution), map.info.width-1)
  cell_y = min(int((y - map.info.origin.position.y) / map.info.resolution), map.info.height-1)
  index = cell_x + cell_y * map.info.width
  map.data[index]

If you have a cell index of the map/grid array and want to know the real world position (x, y) in m:
(The position will be the bottom left corner of the cell. To get the whole area of the cell, expand the position by map.info.resolution in x and in y direction)
  cell_x = int(index % ros_map.info.width)    #[number of cells in x direction]
  cell_y = int(index / ros_map.info.width)    #[number of cells in y direction]
  x = map.info.origin.position.x + cell_x * map.info.resolution
  y = map.info.origin.position.y + cell_y * map.info.resolution

For path planning and dirt generation I recommend using the center of the cells:
The resulting center of cell map.data[index] is in real world:
  cell_x = int(index % ros_map.info.width)    #[number of cells in x direction]
  cell_y = int(index / ros_map.info.width)    #[number of cells in y direction]
  x = map.info.origin.position.x + (cell_x + 0.5) * map.info.resolution
  y = map.info.origin.position.y + (cell_y + 0.5) * map.info.resolution
"""

# IMPORTS
# ---------------------------------------------
import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors

import rospy
import rospkg

from std_msgs.msg import Header, String, Empty
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "map_provider"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables which will be updated/overridden in the inital setup phase or later:
shutdown = False
base_res = 0.0  # m/cell
base_width = 0.0  # cells
base_height = 0.0  # cells
base_origin = Pose()
real_width = 0.0  # m (= base_width * base_res)
real_height = 0.0  # m (= base_height * base_res)

boundary_x_min = 0.0  # m (will be updated in the beginning)
boundary_x_max = 0.0  # m (will be updated in the beginning)
boundary_y_min = 0.0  # m (will be updated in the beginning)
boundary_y_max = 0.0  # m (will be updated in the beginning)

grid_output_path = ""

# CODE
# ---------------------------------------------


def shutdown_callback(message):
    global shutdown
    # message is of type Empty
    shutdown = True


def print_map(ros_map, map_name):
    # Only for testing issues: prints ros_map
    # ros_map is of type OccupancyGrid
    map_array = list(ros_map.data)

    # local parameters:
    occupied_sign = "X"
    occupied_color = '\033[93m'  # '\033[93m' = yellow , '\033[91m' = red
    free_sign = ":"
    free_color = ''  # none/white
    end_color = '\033[0m'  # indicates end of color
    # probably either no spacing (empty string) or a single spacebar char for a bit spacing
    spacing = " "

    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() + ": " + map_name + " is seen by the wrapper as the following one:\n\t(free = '%s' [0], occupied = '%s' [100 or -1]).\n\n\tSize w: %d cells x h: %d cells (%.2f m x %.2f m)\n\tResolution: %.2f m/cell\n\tOrigin: (%.2f , %.2f)" % (
        free_sign, occupied_sign, ros_map.info.width, ros_map.info.height, real_width, real_height, ros_map.info.resolution, ros_map.info.origin.position.x, ros_map.info.origin.position.y))

    array_string = ""
    image_string = ""
    for index in range(0, len(map_array)):
        # Check new line for both strings:
        if index % ros_map.info.width == 0:
            array_string += "\n\t"
            image_string += "\n\t"
        # Indicate symbol for array element:
        if map_array[index] == 0:
            array_string += free_color + ":" + end_color + spacing
        else:
            array_string += occupied_color + "X" + end_color + spacing
        # Indicate symbol for image perspective (translate the index):
        x = int(index % ros_map.info.width)  # x = x_old
        y = (ros_map.info.height - 1) - int(index /
                                            ros_map.info.width)  # y = (height-1) - y_old
        image_index = x + y * ros_map.info.width
        if map_array[image_index] == 0:
            image_string += free_color + ":" + end_color + spacing
        else:
            image_string += occupied_color + "X" + end_color + spacing

    print(
        "\n\tARRAY perspective (based on the actual order of the map/grid array):\n\t[first array element (origin) is top left and then row-major]" + array_string + "\n")
    print(
        "\n\tMAP IMAGE perspective (based on the initial image of the map):\n\t[origin is bottom left and then row-major]" + image_string + "\n")


def save_map_as_image(ros_map, map_name):
    # saves ros_map as grid map image
    # ros_map is of type OccupancyGrid
    array_2d = np.array(ros_map.data).reshape(-1, ros_map.info.width)
    list_2d = np.flip(array_2d, 0).tolist()

    # cmap = colors.ListedColormap(["white", "black"])
    # linear scaling between 0 and 100 (min and max of occupancy grid):
    norm = colors.Normalize(vmin=0, vmax=100)
    figure_height = 10
    resize_factor_height = figure_height / ros_map.info.height
    font_factor = 1.0 + (0.2 / resize_factor_height - 1.0) / 8
    font_size = 18
    plt.figure(figsize=(ros_map.info.width *
                        resize_factor_height, ros_map.info.height * resize_factor_height))
    plt.pcolor(list_2d[::-1], cmap="binary", norm=norm,
               edgecolors="k", linewidths=0.5)
    plt.title(map_name, fontsize=font_size * font_factor, fontweight="bold")
    plt.xlabel("Cell index (of the cell to the left)\nFor real dimensions with the map resolution, multiply with " +
               str(round(ros_map.info.resolution, 4)) + " m/cell", fontsize=(font_size - 4) * font_factor)
    plt.ylabel("Cell index (of the cell underneath)",
               fontsize=(font_size - 4) * font_factor)
    plt.tick_params(axis='both', which='major',
                    labelsize=(font_size - 6) * font_factor)
    # plt.show() # will stop the map_provider until the window is closed!
    plt.savefig(grid_output_path)


def publish_map(publisher, new_map, res, width, height, origin, enable_print, enable_save, first_time):
    # new_map is an array of the map/grid
    ros_map = OccupancyGrid()
    if new_map:  # Only when a new map exists:
        ros_map.data = new_map
        # Edit meta data of new_map
        ros_map.info = MapMetaData()
        ros_map.info.map_load_time = rospy.Time.now()
        ros_map.info.resolution = res
        ros_map.info.height = height
        ros_map.info.width = width
        ros_map.info.origin = origin
        # Edit header
        ros_map.header = Header()
        ros_map.header.stamp = rospy.Time.now()

        # Publish new generated map
        publisher.publish(ros_map)

        # Print and/or save map (if wanted), but only once in the beginning
        if first_time:
            if enable_print:
                map_name = "The transformed map"
                print_map(ros_map, map_name)
            if enable_save:
                map_name = "The transformed map as pure occupancy grid map"
                save_map_as_image(ros_map, map_name)
            first_time = False


def is_occupied(_map, x, y):
    # check if cell at real world position (x,y) is occupied or not (with a static obstacle like a wall)

    # rounding to solve more or less the general float problem (e.g. a 2.0 after division will be represented as 1.9999999...)
    # cell_x = int(round(round(x, 6) / round(_map.info.resolution, 6), 6))
    # cell_y = int(round(round(y, 6) / round(_map.info.resolution, 6), 6))
    cell_x = min(int(round(round(x - _map.info.origin.position.x, 6) /
                           round(_map.info.resolution, 6), 6)), _map.info.width - 1)
    cell_y = min(int(round(round(y - _map.info.origin.position.y, 6) /
                           round(_map.info.resolution, 6), 6)), _map.info.height - 1)
    # since width should be always an integer, the index will be integer, too
    index = cell_x + cell_y * _map.info.width  # because array is in row-major order

    return _map.data[index] != 0


def transform(base_map, new_res, new_width, new_height):
    # Create a new map on basis of the received map (but with other sizes)
    new_map = []  # will be row-major

    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() +
                  " is trying to transform the occupancy grid\n\t(from cell size %.2f m to %.2f m and the map size from %dx%d cells to %dx%d cells)\n" % (base_res, new_res, base_width, base_height, new_width, new_height))

    # 2dim for-loop for indexing the resized cell of the new map (row-major order!)
    # The rows are indexed by y values [y is real distance (in m)]
    for y in np.arange(boundary_y_min, boundary_y_max, new_res):
        # The columns are indexed by x values [x is real distance (in m)]
        for x in np.arange(boundary_x_min, boundary_x_max, new_res):
            # The interesting/selected cell reaches now from x to x+new_res and y to y+new_res
            # The question is now if all "old" cells inside this new cell are free.
            # Then we can also state the new cell as free (0), otherwise we state it as occupied (100), even if only one old cell is occupied
            free = True
            # 2dim for-loop for indexing all cells of the old OccupancyGrid which are inside the new (selected) cell of the new map
            # rounding and '-0.0000001' to solve more or less the general float problem (e.g. a 2.0 after division will be represented as 1.9999999...)
            for j in np.arange(y, round(y + new_res, 6) - 0.0000001, base_res):
                for i in np.arange(x, round(x + new_res, 6) - 0.0000001, base_res):
                    if is_occupied(base_map, i, j):
                        free = False
                        break  # When there is already one occupied old cell, then the checks of the others can be skipped
                if not free:  # Same goes for the other axis loops: When there is already one occupied old cell, then the checks of the others can be skipped
                    break
            # Select the right number based on the state
            if free:
                new_map.append(0)
            else:
                new_map.append(100)

    # New map is completed
    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() +
                  ": Occupancy grid transformation is finished\n\t(cell: from %.2f m to %.2f m --> map: from %dx%d cells (pixels) to %dx%d cells)\n" % (base_res, new_res, base_width, base_height, new_width, new_height))

    return new_map


def map_provider():
    global shutdown, base_res, base_width, base_height, base_origin, real_width, \
        real_height, boundary_x_min, boundary_x_max, boundary_y_min, boundary_y_max, grid_output_path
    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    # Get all needed parameters
    wrapper_namespace = rospy.get_param("/wrapper_namespace")
    # since this node is inside the wrapper namespace, we could also get it
    # with rospy.get_namespace(), but then we would get "/wrapper/" instead of "wrapper"
    provided_map_topic = rospy.get_param(
        "/" + wrapper_namespace + "/provided_map_topic")
    provided_map_path_topic = rospy.get_param(
        "/" + wrapper_namespace + "/provided_map_path_topic")
    final_shutdown_topic = rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    base_map_pkg_name = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_pkg")
    base_map_folder = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_folder")
    map_name = rospy.get_param(
        "/" + wrapper_namespace + "/map_name")
    base_map_topic = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_topic")
    base_map_topic_with_ns = "/" + wrapper_namespace + "/" + base_map_topic
    map_provider_status_topic = rospy.get_param(
        "/" + wrapper_namespace + "/map_provider_status_topic")
    output_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/output_pkg")
    output_map_folder = rospy.get_param(
        "/" + wrapper_namespace + "/output_map_folder")
    grid_file = rospy.get_param(
        "/" + wrapper_namespace + "/grid_file")

    new_res = round(rospy.get_param(
        "/" + wrapper_namespace + "/map_resolution"), 6)  # m/cell (can be chosen be the user)
    # new_origin_dict = rospy.get_param(
    #     "/" + wrapper_namespace + "/new_origin")
    # new_origin = Pose(position=Point(x=new_origin_dict["position"]["x"], y=new_origin_dict["position"]["y"], z=new_origin_dict["position"]["z"]),
    #                   orientation=Quaternion(x=new_origin_dict["orientation"]["x"], y=new_origin_dict["orientation"]["y"], z=new_origin_dict["orientation"]["z"], w=new_origin_dict["orientation"]["w"]))
    print_base_map = rospy.get_param(
        "/" + wrapper_namespace + "/print_base_map")
    print_transformed_map = rospy.get_param(
        "/" + wrapper_namespace + "/print_transformed_map")
    save_grid_map = rospy.get_param(
        "/" + wrapper_namespace + "/save_grid_map")

    use_map_section = rospy.get_param(
        "/" + wrapper_namespace + "/use_map_section")
    map_section_x_min = rospy.get_param(
        "/" + wrapper_namespace + "/map_section_x_min")
    map_section_x_max = rospy.get_param(
        "/" + wrapper_namespace + "/map_section_x_max")
    map_section_y_min = rospy.get_param(
        "/" + wrapper_namespace + "/map_section_y_min")
    map_section_y_max = rospy.get_param(
        "/" + wrapper_namespace + "/map_section_y_max")

    # Publishers
    provided_map_pub = rospy.Publisher(
        provided_map_topic, OccupancyGrid, queue_size=100)
    provided_map_path_pub = rospy.Publisher(
        provided_map_path_topic, String, queue_size=100)
    shutdown_pub = rospy.Publisher(final_shutdown_topic, Empty, queue_size=100)
    status_pub = rospy.Publisher(
        map_provider_status_topic, String, queue_size=100)

    # Subscribers
    shutdown_sub = rospy.Subscriber(
        final_shutdown_topic, Empty, shutdown_callback)

    rospy.loginfo(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
                  f"\n\t...will listen once to '{base_map_topic_with_ns}' for the base map"
                  f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                  f"\n\t...will publish its status to '{status_pub.resolved_name}'"
                  f"\n\t...will publish the absolute path to the image of the map to '{provided_map_path_pub.resolved_name}'"
                  f"\n\t...will publish the provided map to '{provided_map_pub.resolved_name}'\n")

    # get path to the image of the base map:
    rospack = rospkg.RosPack()
    base_map_pkg = rospack.get_path(base_map_pkg_name)
    base_map_file = base_map_pkg + "/" + base_map_folder + \
        "/" + map_name + "/" + map_name + ".pgm"
    base_map_yaml = base_map_pkg + "/" + base_map_folder + \
        "/" + map_name + "/" + map_name + ".yaml"
    map_image_path = String()
    map_image_path.data = base_map_file
    # set it directly as global parameter:
    rospy.set_param("/" + wrapper_namespace +
                    "/base_map_absolute_path", base_map_file)

    green_color = '\033[92m'  # green
    end_color = '\033[0m'  # indicates end of color
    rospy.loginfo(
        "*** WRAPPER MESSAGE ***\n\n\tThe currently used map name is: " + green_color + map_name + end_color + "\n\tPath to map info: '" + green_color + base_map_yaml + end_color + "'\n\tPath to map image: '" + green_color + base_map_file + end_color + "'\n")

    # get image output path:
    output_pkg_path = rospack.get_path(output_pkg)
    grid_output_path = output_pkg_path + "/" + output_map_folder + \
        "/" + grid_file

    # get base map (provided by map_server directly from the provided map image)
    base_map = rospy.wait_for_message(base_map_topic_with_ns, OccupancyGrid)

    base_res = round(base_map.info.resolution, 6)  # m/cell
    base_width = base_map.info.width  # cells
    base_height = base_map.info.height  # cells
    base_origin = base_map.info.origin  # Pose

    # should be same as new_height*new_res
    real_width = round(base_width * base_res, 6)  # m
    # should be same as new_width*new_res
    real_height = round(base_height * base_res, 6)  # m

    if use_map_section:
        # use only the selected section of the image
        # rounding to solve more or less the general float problem (e.g. a 2.0 after division will be represented as 1.9999999...)
        boundary_x_min = round(map_section_x_min, 6)
        boundary_x_max = round(map_section_x_max, 6)
        boundary_y_min = round(map_section_y_min, 6)
        boundary_y_max = round(map_section_y_max, 6)
        # check that it is inside the image:
        if (boundary_x_min < base_origin.position.x
                or boundary_x_max > (base_origin.position.x + real_width)
                or boundary_y_min < base_origin.position.y
                or boundary_y_max > (base_origin.position.y + real_height)):
            rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() + " detected a problem:\n\tThe selected image section is not inside the image!\n"
                         + "\n\tAll of them need to be true (what is not the case):"
                         + "\n\tx_min: %.2f m >= %.2f m" % (boundary_x_min, base_origin.position.x)
                         + "\n\tx_max: %.2f m <= %.2f m" % (boundary_x_max, base_origin.position.x + real_width)
                         + "\n\ty_min: %.2f m >= %.2f m" % (boundary_y_min, base_origin.position.y)
                         + "\n\ty_max: %.2f m <= %.2f m\n" % (boundary_y_max, base_origin.position.y + real_height))
            # stop whole process (inform the other nodes)
            shutdown_pub.publish(Empty())
            shutdown = True
    else:
        # use complete image
        # rounding to solve more or less the general float problem (e.g. a 2.0 after division will be represented as 1.9999999...)
        boundary_x_min = round(base_origin.position.x, 6)  # normally 0.0
        boundary_x_max = round(base_origin.position.x + real_width, 6)
        boundary_y_min = round(base_origin.position.y, 6)  # normally 0.0
        boundary_y_max = round(base_origin.position.y + real_height, 6)

    if print_base_map:
        map_name = "The base map (directly from the map image)"
        print_map(base_map, map_name)

    # the new resolution leads to new cell numbers:
    new_real_width = round(boundary_x_max - boundary_x_min, 6)
    new_real_height = round(boundary_y_max - boundary_y_min, 6)
    new_width = int(round(new_real_width / new_res, 6))  # cells
    new_height = int(round(new_real_height / new_res, 6))  # cells

    # because of the general float problem when dividing small floats (or modulo)
    expander = 100000
    # check resolution
    if ((max(base_res, new_res) * expander) % (min(base_res, new_res) * expander)) != 0:
        rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() + " detected a problem:\n\tThe new resolution does not fit to the one of the old/base map!\n"
                     + "\n\tBase resolution: %.2f m/cell vs new resolution: %.2f m/cell" % (base_res, new_res)
                     + "\n\tThe bigger one needs to be an integer factor of the smaller one.\n\tPlease adjust the wanted resolution (for the transformed map).\n")
        # stop whole process (inform the other nodes)
        shutdown_pub.publish(Empty())
        shutdown = True

    # check the boundary points (if they are perfectly on the grid)
    if ((((boundary_x_min - base_origin.position.x) * expander) % (base_res * expander)) != 0
            or (((boundary_x_max - base_origin.position.x) * expander) % (base_res * expander)) != 0
            or (((boundary_y_min - base_origin.position.y) * expander) % (base_res * expander)) != 0
            or (((boundary_y_max - base_origin.position.y) * expander) % (base_res * expander)) != 0):
        rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() + " detected a problem:\n\tThe new corner points do not fit perfectly on the grid!\n"
                     + "\n\tPossible steps (base resolution): %.2f m/cell" % (base_res)
                     + "\n\tFrom origin to x_min: %.2f m" % (boundary_x_min - base_origin.position.x)
                     + "\n\tFrom origin to x_max: %.2f m" % (boundary_x_max - base_origin.position.x)
                     + "\n\tFrom origin to y_min: %.2f m" % (boundary_y_min - base_origin.position.y)
                     + "\n\tFrom origin to y_max: %.2f m" % (boundary_y_max - base_origin.position.y)
                     + "\n\tAll distances needs to be factors of the resolution/steps.\n\tPlease adjust the wanted resolution (for the transformed map) and/or the image section.\n")
        # stop whole process (inform the other nodes)
        shutdown_pub.publish(Empty())
        shutdown = True

    # if a map section was selected, the origin will be a new one:
    new_origin = Pose(position=Point(x=boundary_x_min, y=boundary_y_min, z=0.0),
                      orientation=base_origin.orientation)

    if not rospy.is_shutdown() and not shutdown:
        # create/transform new map
        new_map = transform(base_map, new_res, new_width, new_height)

    # set map directly as global parameter:
    rospy.set_param("/" + wrapper_namespace +
                    "/transformed_map", new_map)
    rospy.set_param("/" + wrapper_namespace +
                    "/transformed_map_width_cells", new_width)
    rospy.set_param("/" + wrapper_namespace +
                    "/transformed_map_height_cells", new_height)

    rate = rospy.Rate(1)  # Hz
    first_time = True
    while not rospy.is_shutdown() and not shutdown:
        # publish map (and maybe print it the first time)
        publish_map(provided_map_pub, new_map, new_res,
                    new_width, new_height, new_origin, print_transformed_map, save_grid_map, first_time)

        # publish path to image of the initial map
        provided_map_path_pub.publish(map_image_path)

        # signal ready:
        status_pub.publish(String("ready"))

        if first_time:
            first_time = False
        rate.sleep()

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')


if __name__ == '__main__':
    try:
        map_provider()
    except rospy.ROSInterruptException:
        pass
