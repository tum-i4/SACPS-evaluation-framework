#! /usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "09/2020"

"""
This node will create a probability distribution for spawning dirt based on the 
transformed/scaled map (from the map_provider) and inserted hotspots.
Based on this distribution it will generate after a random time a dirt position/object
and publish it. As soon as it is published, the simulator which should be evaluated,
should recognize it and handle it as the dirt is from now on spawned on the map.
"""

# IMPORTS
# ---------------------------------------------
import math
import random
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
from matplotlib.image import NonUniformImage
from matplotlib import colors
from matplotlib import cm

import rospy
import rospkg

from std_msgs.msg import Bool, Header, Empty, String
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from wrapper_msgs.msg import DirtObject, DirtObjectList

# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
# Node name
NODE_NAME = "global_dirt_generator"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables which will be updated in the inital setup phase:
wrapper_namespace = ""  # will be overridden!

# CODE
# ---------------------------------------------


class GlobalDirtGenerator:
    def __init__(self):
        # Get all needed parameters
        self.provided_map_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/provided_map_topic")
        self.start_dirt_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/start_dirt_gen_topic")
        self.final_shutdown_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/final_shutdown_topic")
        self.active_dirt_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/active_dirt_topic")
        self.stop_dirt_gen_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
            "/" + wrapper_namespace + "/stop_dirt_gen_topic")
        self.new_dirt_topic = rospy.get_param(
            "/" + wrapper_namespace + "/new_dirt_topic")  # since this node is publishing it, we only need the name without namespace
        self.global_dirt_generator_status_topic = rospy.get_param(
            "/" + wrapper_namespace + "/global_dirt_generator_status_topic")  # since this node is publishing it, we only need the name without namespace

        self.output_pkg = rospy.get_param(
            "/" + wrapper_namespace + "/output_pkg")
        self.distribution_folder = rospy.get_param(
            "/" + wrapper_namespace + "/distribution_folder")
        self.distribution_image = rospy.get_param(
            "/" + wrapper_namespace + "/distribution_image")

        self.points_per_distr = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_points_per_distr")
        self.strictly_hotspots = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_strictly_hotspots")

        self.prevent_duplicates = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_prevent_duplicates")

        self.enable_printing = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_enable_distr_printing")
        self.enable_plotting = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_enable_distr_plotting")

        self.use_uniform_distr = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_use_uniform_distr")
        self.create_random_hotspots = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_create_random_hotspots")
        self.number_of_random_hotspots = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_number_of_random_hotspots")

        self.hotspots = []
        hotspots_list = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_hotspots")
        for hotspot in hotspots_list:
            self.hotspots.append(
                [hotspot["x"], hotspot["y"], hotspot["spread"]])

        self.seed = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_seed")
        self.time_interval_min = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_time_min")  # in s
        self.time_interval_max = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_time_max")  # in s
        self.min_trust = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_trust_min")  # in %
        self.max_trust = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_trust_max")  # in %

        self.end_after_time = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_end_after_time")
        self.end_after_number = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_end_after_number")

        self.max_time = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_generation_time")  # in s
        self.max_dirt_number = rospy.get_param(
            "/" + wrapper_namespace + "/dirt_generated_number")

        self.robot_size = rospy.get_param(
            "/" + wrapper_namespace + "/robot_size")  # in m

        # tolerance for same dirt position in m, should be half of a cell (= 0.5*resolution)
        self.dirt_pos_tolerance = 0.1  # will be updated as soon as the map is received

        self.start_generation = False
        self.stop_generation = False

        self.dirt_pub = None
        self.status_pub = None
        self.active_dirt_sub = None
        self.shutdown_sub = None
        self.start_gen_sub = None
        self.stop_gen_sub = None

        self.shutdown = False

        # type OccupancyGrid (will be set once in setup_and_start)
        self.occupancy_map = OccupancyGrid()
        self.active_dirt_list = []
        self.probability_distribution_grid = []  # type np.ndarray

        self.__init_publishers()
        self.__init_subscribers()

        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
            f"\n\t...will listen once to '{self.provided_map_topic}' for the provided map"
            f"\n\t...will listen continously to '{self.start_gen_sub.resolved_name}' for the start flag"
            f"\n\t...will listen continously to '{self.active_dirt_sub.resolved_name}' for active dirt elements"
            f"\n\t...will listen continously to '{self.stop_gen_sub.resolved_name}' for stop dirt generation signal"
            f"\n\t...will listen continously to '{self.shutdown_sub.resolved_name}' for shutdown signal"
            f"\n\t...will publish its status to '{self.status_pub.resolved_name}'"
            f"\n\t...will publish new random generated dirt to '{self.dirt_pub.resolved_name}'\n")

    def __init_subscribers(self):
        self.active_dirt_sub = rospy.Subscriber(self.active_dirt_topic,
                                                DirtObjectList, self.__active_dirt_callback)
        self.shutdown_sub = rospy.Subscriber(self.final_shutdown_topic,
                                             Empty, self.__shutdown_callback)
        self.start_gen_sub = rospy.Subscriber(self.start_dirt_topic,
                                              Empty, self.__start_gen_callback)
        self.stop_gen_sub = rospy.Subscriber(self.stop_dirt_gen_topic,
                                             Empty, self.__stop_gen_callback)

    def __init_publishers(self):
        self.dirt_pub = rospy.Publisher(
            self.new_dirt_topic, DirtObject, queue_size=100)
        self.status_pub = rospy.Publisher(
            self.global_dirt_generator_status_topic, String, queue_size=100)

    def __active_dirt_callback(self, message):
        # message is of type DirtObjectList and contains all current active dirt objects fromt he simulator
        self.active_dirt_list = message.dirt_list
        dirt_string = ""
        for dirt in self.active_dirt_list:
            dirt_string += "(x=%.2f,y=%.2f,trust=%d) " % (dirt.pose.position.x,
                                                          dirt.pose.position.y,
                                                          dirt.trust_value)
        if not dirt_string:
            dirt_string = "empty"
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' got active dirt list from simulator: {dirt_string}\n")

    def __shutdown_callback(self, message):
        # message is of type Empty and signals/requests the shutdown of all nodes
        self.shutdown = True

    def __start_gen_callback(self, message):
        # message is of type Empty and signals/requests the start of the dirt generation
        self.start_generation = True
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a start signal.\n\tNode will start generating and publishing dirt.\n")

    def __stop_gen_callback(self, message):
        # message is of type Empty and signals/requests the stop of the dirt generation
        self.stop_generation = True
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a stop signal.\n\tNode will stop generating and publishing dirt.\n")

    def __comparing_points(self, point1, point2) -> bool:
        """
        Compares two Points and returns true if they are identical with some tolerance
        """
        return (abs(point1.x - point2.x) <= self.dirt_pos_tolerance and abs(
            point1.y - point2.y) <= self.dirt_pos_tolerance)

    def __check_for_duplicates(self, point) -> bool:
        """
        Goes through the list with all currently active dirt positions and compare their positions with the given
        position. Returns true if there is a duplicate, false otherwise
        """
        # Check all already published (active) dirt objects (stored and received from the goal_list)
        for dirt in self.active_dirt_list:
            if self.__comparing_points(point, dirt.pose.position):
                return True
        return False

    def __get_cell_index(self, x, y) -> int:
        """
        Translates the given map position into the index of the grid map (array) which represents this position
        [x and y are NOT real positions (in m) but cell numbers/indices]
        """
        # "The map data, in row-major order, starting with (0,0)"
        return x + y * self.occupancy_map.info.width

    def __cell_is_occupied(self, x, y) -> bool:
        """
        Checks if the cell at position (x, y) is occupied (with a static obstacle like a wall)
        [x and y are NOT real positions (in m) but cell numbers/indices]
        """
        return self.occupancy_map.data[self.__get_cell_index(x, y)] != 0

    def __cell_is_in_map(self, x, y) -> bool:
        """
        Checks if the cell at position (x, y) is inside the map (map starts with (0,0) and has width and height)
        [x and y are NOT real positions (in m) but cell numbers/indices]
        """
        return x >= 0 and y >= 0 and x < self.occupancy_map.info.width and y < self.occupancy_map.info.height

    def __has_occupied_neighbors(self, x, y) -> bool:
        """
        Checks if all neighbor cells of the given cell (mask: radius of robot size) are free of STATIC obstacles.
        Mask is probably larger than needed, but it is fine/safer
        [x and y are NOT real positions (in m) but cell numbers/indices]
        """
        # Robot radius in cells (according to OccupancyGrid)
        # robot_size is the diameter (at the biggest place) in m, resolution is m/cell
        robot_radius_in_cells = int(
            math.ceil((self.robot_size / 2) / self.occupancy_map.info.resolution))

        # Only check the cells in the square around the center with edge length of twice the robot_radius
        for r_offset in range(-robot_radius_in_cells, robot_radius_in_cells + 1):
            # Is actually not needed because we assume that the robot is symmetrical (square circle)
            for c_offset in range(-robot_radius_in_cells, robot_radius_in_cells + 1):
                # Now refine the initial square with transforming the mask to a circle (nearly)
                # and the actual cell does not need to be checked (only the neighbors)
                if math.floor(math.sqrt(r_offset ** 2 + c_offset ** 2)) <= robot_radius_in_cells and not (r_offset == 0 and c_offset == 0):
                    selected_cell_x = x + c_offset
                    selected_cell_y = y + r_offset
                    # If the cell is outside of the map or is inside but stated as occupied, it is occupied.
                    # As soon as only one cell is occupied, we can directly return true
                    if not self.__cell_is_in_map(selected_cell_x, selected_cell_y) or self.__cell_is_occupied(selected_cell_x, selected_cell_y):
                        return True

        # Only if all cells in the circle mask are free, the given cell is possible as dirt center and a false can be returned
        return False

    def __print_probability_grid(self, distr_grid):
        """
        Gets the probability distribution of the map and then outputs two strings with separate perspectives.
        1. perspective: in order of the given array (origin is top left)
        2. perspective: in the layout of the initial image (origin is bottom left)
        """

        # local parameters:
        array_string = ""
        image_string = ""
        occupied_value = -1
        occupied_sign = "*"
        occupied_color = '\033[93m'  # yellow
        zero_chance_sign = " "
        busy_cell_color = '\033[91m'  # red
        busy_cell_percentage = 0.5  # everything above is a busy cell and will get the color
        end_color = '\033[0m'  # indicates end of color
        positive_grid = np.ma.masked_where(
            distr_grid <= 0, distr_grid, copy=False)
        min_value = positive_grid.min()
        max_value = positive_grid.max()
        normed_min = 1
        normed_max = 9
        height = distr_grid.shape[0]

        rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id() +
                      ": The general dirt distribution will be like this:\n\t(occupied = '%s', no chance = '%s' and the rest is normed between '%d' and '%d' (highest))" % (occupied_sign, zero_chance_sign, normed_min, normed_max))

        for (cell_y, cell_x), cell_value in np.ndenumerate(distr_grid):
            # Check new line for both strings:
            if cell_x == 0:
                array_string += "\n\t"
                image_string += "\n\t"
            # Indicate symbol for array perspective (directly with given cell_value):
            if cell_value == occupied_value:
                array_string += occupied_color + occupied_sign + end_color + " "
            elif cell_value == 0:
                array_string += zero_chance_sign + " "
            else:
                if min_value == max_value:
                    array_string += str(normed_max) + " "
                else:
                    normed_value = int(round(normed_min +
                                             (cell_value - min_value) *
                                             ((normed_max - normed_min) / (max_value - min_value))))
                    if normed_value >= int(busy_cell_percentage * normed_max):
                        array_string += busy_cell_color + \
                            str(normed_value) + end_color + " "
                    else:
                        array_string += str(normed_value) + " "
            # Indicate symbol for image perspective (swapped on horizontal axis):
            translated_cell_value = distr_grid[(height - 1) - cell_y][cell_x]
            if translated_cell_value == occupied_value:
                image_string += occupied_color + occupied_sign + end_color + " "
            elif translated_cell_value == 0:
                image_string += zero_chance_sign + " "
            else:
                if min_value == max_value:
                    image_string += str(normed_max) + " "
                else:
                    normed_value = int(round(normed_min +
                                             (translated_cell_value - min_value) *
                                             ((normed_max - normed_min) / (max_value - min_value))))
                    if normed_value >= int(busy_cell_percentage * normed_max):
                        image_string += busy_cell_color + \
                            str(normed_value) + end_color + " "
                    else:
                        image_string += str(normed_value) + " "

        print(
            "\n\tARRAY perspective (based on the actual order of the map/grid array):\n\t[first array element (=origin [%.2fm,%.2fm]) is top left and then row-major]" % (self.occupancy_map.info.origin.position.x, self.occupancy_map.info.origin.position.y) + array_string + "\n")
        print(
            "\n\tMAP IMAGE perspective (based on the initial image of the map):\n\t[origin [%.2fm,%.2fm] is bottom left and then row-major]" % (self.occupancy_map.info.origin.position.x, self.occupancy_map.info.origin.position.y) + image_string + "\n")

    def __get_random_hotspot(self):
        """
        Generates a random hotspot based on the size of the map.
        Returns a tuple for a hotspot with x coord., y coord. and spread
        """
        x_min = self.occupancy_map.info.origin.position.x
        x_max = x_min + self.occupancy_map.info.width * self.occupancy_map.info.resolution
        y_min = self.occupancy_map.info.origin.position.y
        y_max = y_min + self.occupancy_map.info.height * \
            self.occupancy_map.info.resolution
        # This might bes a bit strange, but we have the following problem:
        # some simulators need a square version of the same map. A square version
        # will have other x_max or y_max and thus the random hotspots will be different.
        # TO prevent this, we will always take only the max value of either x_max or y_max.
        # This will be the same for the square version and the not-square version (of the same map).
        max_value = max(x_max, y_max)

        # search for a not occupied position
        while True:
            # previously: x = random.uniform(x_min, x_max) # see problem description above
            x = random.uniform(x_min, max_value)
            # previously: y = random.uniform(y_min, y_max) # see problem description above
            y = random.uniform(y_min, max_value)
            # due to the workaround for the problem above, it can be that the value is out
            # of map for the not square map version. We need to skip this (the square
            # map version will skip it due to occupied cell...):
            if x <= x_max and y <= y_max:
                cell_x = min(int(
                    (x - x_min) / self.occupancy_map.info.resolution), self.occupancy_map.info.width - 1)
                cell_y = min(int(
                    (y - y_min) / self.occupancy_map.info.resolution), self.occupancy_map.info.height - 1)
                if not self.__cell_is_occupied(cell_x, cell_y):
                    break
        spread = random.uniform(0.5, 1.0)
        return (x, y, spread)

    def __generate_distribution_grid(self) -> np.ndarray:
        """
        Generates a probability distribution for the given grid/map based on the given hotspots.
        Returns a 2d array (in map layout) with probabilities for each cell (starting at 0,0 and going in row-major order -> all x in each row and then next y-row)
        """

        x_min = self.occupancy_map.info.origin.position.x
        x_max = x_min + self.occupancy_map.info.width * self.occupancy_map.info.resolution
        y_min = self.occupancy_map.info.origin.position.y
        y_max = y_min + self.occupancy_map.info.height * \
            self.occupancy_map.info.resolution

        # CREATE HOTSPOTS IF WANTED:
        if self.create_random_hotspots:
            if self.use_uniform_distr:
                rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id()
                             + ":\n\tIt does not make sense to create random hotspots if distribution should be uniform."
                             + "\n\tThe node will go on with uniform distribution. You can change this in the launch parameters.\n")
                self.number_of_random_hotspots = 1  # still needed to get the right edges
            elif self.number_of_random_hotspots <= 0:
                rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id()
                             + ":\n\tThe number of wanted hotspots is 0 or below. This is not possible."
                             + "\n\tThe node will go on with 1 random hotspot. You can change this in the launch parameters.\n")
                self.number_of_random_hotspots = 1
            self.hotspots = []
            for _ in range(self.number_of_random_hotspots):
                x, y, spread = self.__get_random_hotspot()
                self.hotspots.append([x, y, spread])

        if len(self.hotspots) < 1:
            if not self.use_uniform_distr:
                rospy.logerr("*** WRAPPER MESSAGE ***\n\n\t" + rospy.get_caller_id()
                             + ":\n\tIf no hotspots should be generated randomly (and no uniform distribution),\n\tyou need to specify hotspots."
                             + "\n\tThe node will go on with 1 random hotspot. You can change this in the launch parameters.\n")
            x, y, spread = self.__get_random_hotspot()
            self.hotspots = [[x, y, spread]]

        hotspots_string = "\n\tDirt generator - HOTSPOTS information:"
        if self.use_uniform_distr:
            hotspots_string += "\n\n\tThe dirt distribution was set to be uniform. Hotspots do not exist."
        else:
            if self.create_random_hotspots:
                hotspots_string += "\n\n\tThe dirt distribution is based on the " + str(len(
                    self.hotspots)) + " randomly created hotspot(s) [real world position in m]:"
            else:
                hotspots_string += "\n\tThe dirt distribution is based on the " + \
                    str(len(self.hotspots)) + \
                    " specified hotspot(s) [real world position in m]:"
            for index, hotspot in enumerate(self.hotspots):
                hotspots_string += "\n\t\tHotspot %d: x=%.2f, y=%.2f, spread=%.2f" % (
                    index + 1, hotspot[0], hotspot[1], hotspot[2])
        print(hotspots_string + "\n")

        # CREATING SEVERAL NORMAL DISTRIBUTIONS:
        # creating for each hotspot one normal distribution and then combine them to one for X and one for Y
        # for X:
        normal_distributions_attr_X = []
        for spot in self.hotspots:
            # each with [mean_x, std. deviation]
            normal_distributions_attr_X.append([spot[0], spot[2]])
        X_distributions = []
        for attributes in normal_distributions_attr_X:
            distr = stats.truncnorm((x_min - attributes[0]) / attributes[1], (
                x_max - attributes[0]) / attributes[1], loc=attributes[0], scale=attributes[1])
            X_distributions.append(distr.rvs(self.points_per_distr))
        X = np.concatenate(X_distributions)
        # to ensure that the distribution and later the grid is full sized (at a point at each end for full axis):
        X = np.append(X, [x_min, x_max])
        # for Y:
        normal_distributions_attr_Y = []
        for spot in self.hotspots:
            # each with [mean_y, std. deviation]
            normal_distributions_attr_Y.append([spot[1], spot[2]])
        Y_distributions = []
        for attributes in normal_distributions_attr_Y:
            distr = stats.truncnorm((y_min - attributes[0]) / attributes[1], (
                y_max - attributes[0]) / attributes[1], loc=attributes[0], scale=attributes[1])
            Y_distributions.append(distr.rvs(self.points_per_distr))
        Y = np.concatenate(Y_distributions)
        # to ensure that the distribution and later the grid is full sized (at a point at each end for full axis):
        Y = np.append(Y, [y_min, y_max])

        # # Plotting of the two separate distributions (rather for testing)
        # fig, ax = plt.subplots(2, sharex=True)
        # ax[0].hist(X, bins=self.occupancy_map.info.width)
        # ax[1].hist(Y, bins=self.occupancy_map.info.height)
        # plt.show() # will block the remaining process!

        # COMBINING TWO DISTRIBUTIONS TO A BIVARIATE DISTRIBUTION:
        H, xedges, yedges = np.histogram2d(
            X, Y, bins=[self.occupancy_map.info.width, self.occupancy_map.info.height], density=False)
        H = H.astype('int32')
        # inverse H, otherwise x and y would be the other way around (now, x are the columns, and y are rows, with row-major ordered)
        H = H.T
        # H starts with its inital element in (x_min, y_min) and then row-major ordered (see above): each row is one y and in there are all the column-x for this y

        if not self.strictly_hotspots:
            # if all cells should have at least a very small chance, no cell should have 0, which is why we add 1 to all of them
            H = H + 1

        if self.use_uniform_distr:
            H = np.ones((self.occupancy_map.info.height,
                         self.occupancy_map.info.width), dtype=int)

        # already kick out all static occupied cells (walls, etc.) and replace them with -1
        for (cell_y, cell_x), _ in np.ndenumerate(H):
            if self.__cell_is_occupied(cell_x, cell_y):
                H[cell_y][cell_x] = -1

        if self.enable_printing:
            self.__print_probability_grid(H)

        # Creating image
        max_tick_number = 10
        x_axis_res = self.occupancy_map.info.resolution
        while (x_max - x_min) / x_axis_res > max_tick_number:
            x_axis_res *= 2
        x_axis = np.arange(
            x_min, x_max + self.occupancy_map.info.resolution, x_axis_res)
        y_axis_res = self.occupancy_map.info.resolution
        while (y_max - y_min) / y_axis_res > max_tick_number:
            y_axis_res *= 2
        y_axis = np.arange(
            y_min, y_max + self.occupancy_map.info.resolution, y_axis_res)

        figure_height = 10
        resize_factor_height = figure_height / self.occupancy_map.info.height
        fontsize = 26
        font_factor = 1.0 + (0.2 / resize_factor_height - 1.0) / 8

        fig = plt.figure(figsize=(2 * self.occupancy_map.info.width *
                                  resize_factor_height, self.occupancy_map.info.height * resize_factor_height))
        title = "Dirt probability distribution in perspective of the initial map image"
        fig.suptitle(title, fontsize=fontsize * font_factor, fontweight="bold")

        viridis = cm.get_cmap('autumn_r', 256)
        newcolors = viridis(np.linspace(0, 1, 256))
        black = np.array([0, 0, 0, 1])
        white = np.array([1, 1, 1, 1])
        newcolors[:1, :] = black
        newcolors[1:20, :] = white
        cmap = colors.ListedColormap(newcolors)

        # actual bins/edges:
        ax = fig.add_subplot(1, 2, 1, aspect="equal")
        ax.set_title("Actual cells", size=(fontsize - 2)
                     * font_factor, fontweight="bold")
        mesh_X, mesh_Y = np.meshgrid(xedges, yedges)
        ax.pcolormesh(mesh_X, mesh_Y, H, cmap=cmap,
                      edgecolors="k", linewidths=0.5)
        ax.set_xlabel("X axis [m]", fontsize=(fontsize - 4) * font_factor)
        ax.set_ylabel("Y axis [m]", fontsize=(fontsize - 4) * font_factor)
        ax.set_xticks(x_axis)
        ax.set_yticks(y_axis)

        plt.tick_params(axis='both', which='major',
                        labelsize=(fontsize - 6) * font_factor)

        # interpolated:
        ax = fig.add_subplot(1, 2, 2, aspect="equal",
                             xlim=xedges[[0, -1]], ylim=yedges[[0, -1]])
        ax.set_title("Interpolated", size=(fontsize - 2)
                     * font_factor, fontweight="bold")
        interpolated_image = NonUniformImage(
            ax, interpolation="bilinear", cmap=cmap)
        xcenters = (xedges[:-1] + xedges[1:]) / 2
        ycenters = (yedges[:-1] + yedges[1:]) / 2
        interpolated_image.set_data(xcenters, ycenters, H)
        ax.images.append(interpolated_image)
        ax.set_xlabel("X axis [m]", fontsize=(fontsize - 4) * font_factor)
        ax.set_ylabel("Y axis [m]", fontsize=(fontsize - 4) * font_factor)
        ax.set_xticks(x_axis)
        ax.set_yticks(y_axis)

        plt.tick_params(axis='both', which='major',
                        labelsize=(fontsize - 6) * font_factor)

        # get path to the output file
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(self.output_pkg)
        image_path = pkg_path + "/" + self.distribution_folder + \
            "/" + self.distribution_image

        plt.savefig(image_path)

        if self.enable_plotting:
            warning = "Do not close this window until the end. Otherwise the generator node could crash!"
            fig = plt.gcf()
            fig.canvas.set_window_title(warning)
            # show it (without blocking the remaining process)
            plt.ion()
            # plt.draw()
            plt.pause(0.001)
            # but cannot be closed without crashing this node!

        # each cell should have the probability of itself regarding the final distribution:
        # the occupied cells will have -1 and we do not want negative probabilities. So, we replace them with 0
        for (cell_y, cell_x), cell_value in np.ndenumerate(H):
            if cell_value < 0:
                H[cell_y][cell_x] = 0
        prob_grid = H / np.sum(H)

        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has created the final probability distribution.\n"
            + "\tAn image of it was saved to (will be overridden each time!):\n"
            + image_path + "\n")

        return prob_grid

    def __generate_point_based_on_prob(self) -> Point:
        """
        Generates a random point based on probabilities (map/distribution)
        Returns random point of type Point(x, y, z)
        """
        possible = False
        while not possible:
            # make the random decision based on a distribution (hot spots / different probabilities)
            prob_list = self.probability_distribution_grid.flatten()
            selected_index = np.random.choice(
                np.arange(0, len(prob_list)), p=prob_list)

            # get the indices of the cell (from the one array index)
            # width is the number of cells in x directions (it starts with cell 0/0) and is needed due to row-major order
            cell_x = int(selected_index % self.occupancy_map.info.width)
            cell_y = int(selected_index / self.occupancy_map.info.width)

            # get the real world coordinates (which represents the center of the cell)
            x = self.occupancy_map.info.origin.position.x + \
                (cell_x + 0.5) * self.occupancy_map.info.resolution
            y = self.occupancy_map.info.origin.position.y + \
                (cell_y + 0.5) * self.occupancy_map.info.resolution

            # Check if the actual cell is free of STATIC obstacles (not occupied)
            if not self.__cell_is_occupied(cell_x, cell_y):
                # Check for not occupied neighbors (the robot needs some space the reach it)
                if not self.__has_occupied_neighbors(cell_x, cell_y):
                    # If actual spawning of dirt is enabled, then it should also be ensured that no other dirt object is already
                    # at this position, because spawning a model in the same location of an already existing model can lead to problems
                    if not self.prevent_duplicates or not self.__check_for_duplicates(Point(x, y, 0.0)):
                        possible = True
                    else:
                        rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\tGenerated dirt at (%.2f | %.2f) was refused due to already "
                                      "active dirt at this position.\n\tGenerating next one...\n" % (x, y))
                else:
                    rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\tGenerated dirt at (%.2f | %.2f) was refused due to occupied neighbor "
                                  "cells.\n\tGenerating next one...\n" % (x, y))
            else:
                rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\tGenerated dirt at (%.2f | %.2f) was refused due to occupied cell."
                              "\n\tGenerating next one...\n" % (x, y))
        return Point(x=x, y=y, z=0.0)

    def __publish_dirt(self, dirt):
        """
        Publishs the new generated DirtObject
        """
        self.dirt_pub.publish(dirt)

    def generation_process(self):
        """
        Creates a random dirt until the node is shut down
        or another end requirement is reached (time or dirt number)
        """
        start_time = rospy.get_time()
        end_time = start_time + self.max_time
        index = 0
        while not rospy.is_shutdown() and not self.shutdown and not self.stop_generation:
            # check time or dirt number (if this is a requirement) for termination criteria
            if self.end_after_time:
                current_time = rospy.get_time()
                if current_time > end_time:
                    self.shutdown = True
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has reached the maximum generation time:\n\t({current_time} s > {end_time} s).\n\tNode will stop generating and publishing dirt.\n")
                    break
            if self.end_after_number:
                # this will only be important in the beginning if max_dirt_number=0
                # (otherwise the number check after publishing will always trigger first)
                if index >= self.max_dirt_number:
                    self.shutdown = True
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has reached the maximum dirt number ({self.max_dirt_number}).\n\tNode will stop generating and publishing dirt.\n")
                    break

            # Create an (increasing) index, a random trust value and a random position for the new dirt
            header = Header()
            header.stamp = rospy.Time.now()
            index += 1
            trust_value = random.randint(
                self.min_trust, self.max_trust)
            pose = Pose(position=self.__generate_point_based_on_prob(),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            sleep_time = random.randint(
                self.time_interval_min, self.time_interval_max)

            # Combine everything
            dirt = DirtObject(header, index, pose, trust_value)

            # Publish the dirt
            self.__publish_dirt(dirt)

            rospy.loginfo("*** WRAPPER MESSAGE ***\n\n\tDirt was generated and publised: [ID: %d, position: (%.2f,%.2f), trust: %d]\n\tDirt generation will sleep now for %d seconds.\n" % (
                dirt.id, dirt.pose.position.x, dirt.pose.position.y, dirt.trust_value, sleep_time))

            # check dirt number (if this is a requirement)
            if self.end_after_number:
                if index >= self.max_dirt_number:
                    self.shutdown = True
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has reached the maximum dirt number ({self.max_dirt_number}).\n\tNode will stop generating and publishing dirt.\n")
                    break

            # Sleep rest of the (random defined) time
            rospy.sleep(sleep_time)

        # State some final values after stopping generation
        duration = rospy.get_time() - start_time
        duration_string = "%.2f" % duration
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has generated {index} dirt in total over {duration_string} s.\n")

    def setup_and_start(self):
        """
        Set ups everything needed for dirt generation (map as basis and parameters).
        Then waits for the start signals and starts generating dirt (and publishs it)
        """
        # Setup:
        # Map...
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is waiting for the provided occupancy map.\n")
        self.occupancy_map = rospy.wait_for_message(
            self.provided_map_topic, OccupancyGrid)

        # Parameters...
        self.dirt_pos_tolerance = 0.5 * self.occupancy_map.info.resolution
        random.seed(self.seed)
        np.random.seed(self.seed)

        # Create distribution
        self.probability_distribution_grid = self.__generate_distribution_grid()

        rospy.sleep(2)

        # signal ready:
        self.status_pub.publish(String("ready"))

        # Wait for start signal and then start generation
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is waiting for signal to start generating dirt.\n")
        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown() and not self.shutdown and not self.stop_generation:
            if self.start_generation:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' starts generating dirt.\n")
                self.generation_process()
                break
            rate.sleep()

        if self.enable_plotting:
            plt.close("all")

        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
        rospy.signal_shutdown('Intentional shutdown')


if __name__ == '__main__':
    try:
        # Init node
        rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

        # Get all needed parameters
        wrapper_namespace = rospy.get_param("/wrapper_namespace")
        # since this node is inside the wrapper namespace, we could also get it
        # with rospy.get_namespace(), but then we would get "/wrapper/" instead of "wrapper"

        generator = GlobalDirtGenerator()

        generator.setup_and_start()

    except rospy.ROSInterruptException:
        pass
