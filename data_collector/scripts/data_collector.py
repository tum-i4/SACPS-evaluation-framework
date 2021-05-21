#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "12/2020"

"""
This node will collect all needed data for the evaluation process
by listening to several pre-defined topics on which the simulator
should write all its output/information.
All elements will be stored in a panda dataframe, which will be
send to the evaluator in the end.
"""

# IMPORTS
# ---------------------------------------------
import sys
import os.path
import math

import numpy as np
import pandas as pd
from pydoc import locate

from matplotlib import pyplot as plt
from matplotlib import colors

import imageio
import cv2

from copy import deepcopy

import rospy
import rospkg

# complete package because we do not know which message types will be set
import std_msgs.msg
# from std_msgs.msg import Empty, String  # to be able to ignore the namespace
# complete package because we do not know which message types will be set
import geometry_msgs.msg
# complete package because we do not know which message types will be set
import nav_msgs.msg
from nav_msgs.msg import OccupancyGrid


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "data_collector"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

time_name = ""  # will get overridden

shutdown = False
stop_collecting = False
row_list = []
columns = []
latest_outputs = {}
simulation_clock = 0

status_pub = None

undetected_dirt_color = "violet"
detected_dirt_color = "yellow"
robot_colors = ["tab:red", "tab:blue",
                "tab:orange", "tab:green", "lime", "aqua", "mediumvioletred"]

image_counter = 0
file_paths = []

# The following one will be overridden:
robot_size = 0.0
robot_count = 0
occupancy_map = None
tmp_folder_path = ""
all_location_attributes = []
font_size = 0
font_factor = 0

# CODE
# ---------------------------------------------


def simulation_clock_callback(message):
    global simulation_clock
    simulation_clock = message.data


def shutdown_callback(message):
    global shutdown
    # message is of type Empty
    shutdown = True


def stop_collector_callback(message):
    global stop_collecting
    # message is of type Empty and signals/requests the stop of the dirt generation
    stop_collecting = True
    status_pub.publish(std_msgs.msg.String("saving_data"))
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a stop signal.\n\tNode will stop collecting simulation data.\n")


def trigger_callback(message):
    global latest_outputs, row_list, columns
    # message is of type Empty
    # it signals that the latest output values should be saved in the dictionary,
    # from which the panda dataframe will be created in the end
    if not stop_collecting:
        latest_outputs[time_name] = simulation_clock
        new_row = dict(
            (data_name, latest_outputs[data_name]) for data_name in columns)
        row_list.append(new_row)
        # print(str(simulation_clock) + "s: " + str(row_list))


def ros_to_python(ros_msg):
    # some basic ros msg types are not directly integers, etc.
    # they have often containers like "data", which we actually want:
    if isinstance(ros_msg, (std_msgs.msg.Int8, std_msgs.msg.Int16, std_msgs.msg.Int32, std_msgs.msg.Int64,
                            std_msgs.msg.Float32, std_msgs.msg.Float64, std_msgs.msg.String, std_msgs.msg.Bool,
                            std_msgs.msg.Float32MultiArray, std_msgs.msg.Float64MultiArray)):
        return ros_msg.data
    elif isinstance(ros_msg, std_msgs.msg.Empty):
        return "empty_signal"
    else:
        return ros_msg  # is str(ros_msg) better?


def callback_creator(output_name):
    # can dynamically create callback functions for data topics (during run-time)
    def _function(message):
        global latest_outputs
        # By default, callbacks will be executed in sequence ("callback queues").
        # So, we can always access the same global variable in all callbacks
        # without fearing that a parallel overrite can happen!
        if not stop_collecting:
            # print("Received: '" + str(message) + "'")
            converted_msg = ros_to_python(message)
            # print("Converted to: '" + str(converted_msg) +
            #       "' of type: " + str(type(converted_msg)))
            latest_outputs[output_name] = converted_msg
            # Now all defined outputs will be received and the lastest messages will be stored inside one dictionary

    return _function


def create_patch(x, y, size, p_type="robot", index=0):
    index = min(index, robot_count - 1)
    if p_type == "goal":
        return plt.Circle((x, y), size / 2, linewidth=2, edgecolor=robot_colors[index], facecolor="black", label="goal")
    elif p_type == "undetected_dirt":
        return plt.Circle((x, y), size / 2, linewidth=1, edgecolor="black", facecolor=undetected_dirt_color, label="undetected dirt")
    elif p_type == "detected_dirt":
        return plt.Circle((x, y), size / 2, linewidth=1, edgecolor="black", facecolor=detected_dirt_color, label="detected dirt")
    else:
        return plt.Circle((x, y), size / 2, color=robot_colors[index], label="robot")


def create_all_locations(ax, ros_map, row, location_attributes):
    location_name, p_type, size = location_attributes
    tmp_artists = []
    location_list = np.array(row[location_name])
    location_list = location_list.reshape((int(len(location_list) / 2), 2))
    for index, location in enumerate(location_list):
        # a perfect cell index (integer) is not necessary here
        cell_x = (location[0] - ros_map.info.origin.position.x) / \
            ros_map.info.resolution
        # a perfect cell index (integer) is not necessary here
        cell_y = (location[1] - ros_map.info.origin.position.x) / \
            ros_map.info.resolution
        # ax.draw_artist(create_patch(cell_x, cell_y, size, p_type, index))
        tmp_artists.append(ax.add_patch(
            create_patch(cell_x, cell_y, size, p_type, index)))
    return tmp_artists


def create_image_from_row(row, background, index, index_max):
    # takes the most important locations from the given recorded data row
    # and creates an image of it (on the given background, which should be the base map)
    fig = plt.gcf()
    ax = fig.gca()

    # restore the figure with the background to not use the old drawn locations
    fig.canvas.restore_region(background)

    # go through all locations and draw them
    tmp_artists = []
    for location_attributes in all_location_attributes:
        # create_all_locations(ax, occupancy_map, row, location_attributes)
        tmp_artists.extend(create_all_locations(
            ax, occupancy_map, row, location_attributes))
    # add timestamp
    time_string = "Time:\n" + \
        str(round(row["theoretical_elapsed_time"], 2)) + " s"
    text_artist = fig.text(0.85, 0.5, time_string,
                           fontsize=font_size * font_factor, weight="bold")

    # only redraw the new generated elements/artists (huge performance boost!)
    ax.draw_artist(text_artist)
    for artist in tmp_artists:
        ax.draw_artist(artist)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()

    # convert it into array, because this saves much time for generating the gif later
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    steps = 10
    if (index % math.ceil((index_max / steps))) == 0:
        print("\t" + str(index + 1) + " of " + str(index_max) +
              " images created for the final simulation history gif.")
    return image


def set_up_image_creation():
    global all_location_attributes, font_size, font_factor
    # cell_object_center_offset = 0.5  # center of a cell is its half
    robot_cell_size = robot_size / occupancy_map.info.resolution
    goal_size = 1.0  # of cell size
    dirt_size = 0.6  # of cell size

    # ros_map is of type OccupancyGrid
    array_2d = np.array(occupancy_map.data).reshape(-1,
                                                    occupancy_map.info.width)
    list_2d = np.flip(array_2d, 0).tolist()
    # linear scaling between 0 and 100 (min and max of occupancy grid):
    norm = colors.Normalize(vmin=0, vmax=100)
    figure_height = 8
    resize_factor_height = figure_height / occupancy_map.info.height
    font_factor = 1.0 + (0.2 / resize_factor_height - 1.0) / 8
    font_size = 18
    legend_width_coorection = 3

    fig = plt.figure(figsize=(occupancy_map.info.width *
                              resize_factor_height + legend_width_coorection, occupancy_map.info.height * resize_factor_height), tight_layout=True)
    plt.title("Simulation progress", fontsize=font_size *
              font_factor, fontweight="bold")
    plt.xlabel("Cell index (of the cell to the left)\nFor real dimensions with the map resolution, multiply with " +
               str(round(occupancy_map.info.resolution, 4)) + " m/cell", fontsize=(font_size - 4) * font_factor, weight="bold")
    plt.ylabel("Cell index (of the cell underneath)",
               fontsize=(font_size - 4) * font_factor, weight="bold")
    plt.tick_params(axis='both', which='major',
                    labelsize=(font_size - 6) * font_factor)

    # Base map for every image:
    plt.pcolor(list_2d[::-1], cmap="binary", norm=norm,
               edgecolors="k", linewidths=0.5)

    legend_handles = [
        create_patch(0, 0, robot_cell_size, "robot"),
        create_patch(0, 0, goal_size, "goal"),
        create_patch(0, 0, dirt_size, "undetected_dirt"),
        create_patch(0, 0, dirt_size, "detected_dirt")
    ]
    plt.rcParams['legend.title_fontsize'] = font_size * font_factor
    plt.legend(legend_handles, ("robot location", "next goal for\nrelated robot", "undetected dirt", "detected dirt"), title="Legende:",
               fancybox=True, bbox_to_anchor=(1.001, 1), loc='upper left', prop=dict(size=(font_size - 2) * font_factor, weight="bold"))

    all_location_attributes = [
        ["current_robot_locations", "robot", robot_cell_size],
        ["current_goals_locations", "goal", goal_size],
        ["current_undetected_dirt_locations", "undetected_dirt", dirt_size],
        ["current_detected_dirt_locations", "detected_dirt", dirt_size]
    ]

    # draw it once so that it is rendered and can be saved as static background
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(fig.bbox)
    return background


def create_simulation_gif(output_path):
    # saves a gif which has an image for every recorded timestamp
    plt.close()

    # build gif
    with imageio.get_writer(output_path, mode='I') as writer:
        for file_path in file_paths:
            image = imageio.imread(file_path)
            writer.append_data(image)

    # Remove files (can be commented out if images should persist and be overridden each time)
    # for file_path in set(file_paths):
    #     os.remove(file_path)


def data_collector():
    global columns, latest_outputs, time_name, status_pub, robot_size, robot_count, occupancy_map, tmp_folder_path
    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    # Get all needed parameters
    wrapper_namespace = rospy.get_param("/wrapper_namespace")
    active_wrapper = rospy.get_param(
        "/" + wrapper_namespace + "/active_wrapper")
    final_shutdown_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    data_collector_status_topic = rospy.get_param(
        "/" + wrapper_namespace + "/data_collector_status_topic")
    output_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/output_pkg")
    recorded_data_folder = rospy.get_param(
        "/" + wrapper_namespace + "/recorded_data_folder")
    recorded_data_csv = rospy.get_param(
        "/" + wrapper_namespace + "/recorded_data_csv")
    stop_data_collector_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/stop_data_collector_topic")
    mandatory_data_output = rospy.get_param(
        "/" + wrapper_namespace + "/mandatory_data_output")
    mandatory_data_output_order = rospy.get_param(
        "/" + wrapper_namespace + "/mandatory_data_output_order")
    optional_data_output = rospy.get_param(
        "/" + wrapper_namespace + "/optional_data_output")
    optional_data_output_order = rospy.get_param(
        "/" + wrapper_namespace + "//optional_data_output_order")
    recording_trigger_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/recording_trigger_topic")
    simulation_clock_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/simulation_clock_topic")
    time_name = rospy.get_param("/" + wrapper_namespace + "/time_name")
    print_row_index = rospy.get_param(
        "/" + wrapper_namespace + "/print_row_index")
    params_to_save = rospy.get_param(
        "/" + wrapper_namespace + "/params_to_save")
    metadata_text = rospy.get_param(
        "/" + wrapper_namespace + "/metadata_text")
    config_save_folder = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_folder")
    config_save_file = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_file")
    run_id = rospy.get_param(
        "/" + wrapper_namespace + "/run_id")
    simulation_history_folder = rospy.get_param(
        "/" + wrapper_namespace + "/simulation_history_folder")
    simulation_history_file = rospy.get_param(
        "/" + wrapper_namespace + "/simulation_history_file")
    provided_map_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/provided_map_topic")
    robot_size = rospy.get_param(
        "/" + wrapper_namespace + "/robot_size")  # in m
    robot_count = rospy.get_param(
        "/" + wrapper_namespace + "/sim_no_of_robots")
    save_sim_history = rospy.get_param(
        "/" + wrapper_namespace + "/save_sim_history")
    sim_history_fps = rospy.get_param(
        "/" + wrapper_namespace + "/sim_history_fps")

    if active_wrapper:
        # It is faster to add dynamically the new rows as dicts in a list and
        # translate it in the end once into a panda dataframe instead of
        # extending a dataframe all the time:
        # https://stackoverflow.com/a/47979665

        if robot_count > len(robot_colors):
            rospy.logerr("\nThe number of robots is " + str(robot_count) + ", but only " + str(len(robot_colors)) +
                         " colors are specified inside data_collector.py for the map history gif. A gif will not be produced!\n")

        # Subscribers
        shutdown_sub = rospy.Subscriber(
            final_shutdown_topic, std_msgs.msg.Empty, shutdown_callback)
        stop_collector_sub = rospy.Subscriber(stop_data_collector_topic,
                                              std_msgs.msg.Empty, stop_collector_callback)
        trigger_sub = rospy.Subscriber(recording_trigger_topic,
                                       std_msgs.msg.Empty, trigger_callback)
        simulation_clock_sub = rospy.Subscriber(simulation_clock_topic,
                                                std_msgs.msg.Int32, simulation_clock_callback)

        # Setting up the table (columns, ...):
        additional_columns = [time_name]  # currently only the sim time

        mandatory_column_count = len(mandatory_data_output)
        if len(mandatory_data_output_order) > mandatory_column_count:
            rospy.logerr("\nThe number of keys in parameter 'mandatory_data_output_order' is higher then the actual keys in 'mandatory_data_output'.\n"
                         + "It is now very likely that the creation of the recording table will throw errors!\n")
            # if it is less, it does not matter
        mandatory_columns = ["" for _ in range(mandatory_column_count)]
        data_subscribers = []
        remaining_mandatory_keys = []
        for key, value in mandatory_data_output.items():
            # print(locate(value["type"]))
            data_subscribers.append(rospy.Subscriber(value["topic"],
                                                     locate(value["type"]), callback_creator(key)))
            if key in mandatory_data_output_order:
                order_index = mandatory_data_output_order.index(key)
                if order_index >= mandatory_column_count:
                    rospy.logerr("\nThe key '" + key + "' has a higher order index in 'mandatory_data_output_order' than there are keys in 'mandatory_data_output'.\n"
                                 + "If there are placeholders left in the end, it will be added there, but otherwise the key will be ignored!\n")
                    remaining_mandatory_keys.append(key)
                else:
                    mandatory_columns[order_index] = key
            else:
                remaining_mandatory_keys.append(key)
        for key in remaining_mandatory_keys:
            # replace the remaining placeholders "" with the remaining keys
            if "" in mandatory_columns:
                mandatory_columns[mandatory_columns.index("")] = key
            else:
                mandatory_columns.append(key)

        optional_column_count = len(
            optional_data_output) - 1  # -1 because of "dummy"
        if len(optional_data_output_order) > optional_column_count:
            rospy.logerr("\nThe number of keys in parameter 'optional_data_output_order' is higher then the actual keys in 'optional_data_output'.\n"
                         + "It is now very likely that the creation of the recording table will throw errors!\n")
        optional_columns = ["" for _ in range(optional_column_count)]
        remaining_optional_keys = []
        if optional_data_output:
            for key, value in optional_data_output.items():
                if key != "dummy":
                    data_subscribers.append(rospy.Subscriber(value["topic"],
                                                             locate(value["type"]), callback_creator(key)))
                    if key in optional_data_output_order:
                        order_index = optional_data_output_order.index(key)
                        if order_index >= optional_column_count:
                            rospy.logerr("\nThe key '" + key + "' has a higher order index in 'optional_data_output_order' than there are keys in 'optional_data_output'.\n"
                                         + "If there are placeholders left in the end, it will be added there, but otherwise the key will be ignored!\n")
                            remaining_optional_keys.append(key)
                        else:
                            optional_columns[order_index] = key
                    else:
                        remaining_optional_keys.append(key)
        for key in remaining_optional_keys:
            # replace the remaining placeholders "" with the remaining keys
            if "" in optional_columns:
                optional_columns[optional_columns.index("")] = key
            else:
                optional_columns.append(key)

        columns = additional_columns + mandatory_columns + optional_columns

        # Publishers
        status_pub = rospy.Publisher(
            data_collector_status_topic, std_msgs.msg.String, queue_size=100)

        # get path to the output files
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(output_pkg)
        records_path = pkg_path + "/" + recorded_data_folder + \
            "/" + recorded_data_csv
        config_save_path = pkg_path + "/" + config_save_folder + \
            "/" + config_save_file
        simulation_history_save_path = pkg_path + "/" + simulation_history_folder + \
            "/" + simulation_history_file
        tmp_folder_path = pkg_path + "/" + simulation_history_folder + "/tmp/"

        subscriber_topics = [sub.resolved_name for sub in data_subscribers]
        rospy.loginfo(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
                      f"\n\t...will publish its status to '{status_pub.resolved_name}'"
                      f"\n\t...will listen continously to '{simulation_clock_sub.resolved_name}' for simulation clock"
                      f"\n\t...will listen continously to '{stop_collector_sub.resolved_name}' for collection stop signal"
                      f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                      f"\n\t...will listen continously to '{trigger_sub.resolved_name}' for recording trigger signal"
                      f"\n\t...will record these data points (each time a trigger signal is received):\n\t{columns}"
                      f"\n\t...by listening to these topics:\n\t{subscriber_topics}\n")

        for data_name in columns:
            latest_outputs[data_name] = None

        occupancy_map = rospy.wait_for_message(
            provided_map_topic, OccupancyGrid)

        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has started the data collection process.\n")

        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown() and not shutdown and not stop_collecting:
            status_pub.publish(std_msgs.msg.String("ready"))
            rate.sleep()

        # stop collecting and save data
        status_pub.publish(std_msgs.msg.String("saving_data"))
        for sub in data_subscribers:
            sub.unregister()
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is saving the recorded data in:\n\t" + records_path + "\n")
        records_df = pd.DataFrame(row_list, columns=columns)
        records_df.to_csv(records_path, index=print_row_index)

        # save config
        param_values = [rospy.get_param(
            "/" + wrapper_namespace + "/" + param) for param in params_to_save]
        all_names = ["metadata_description", "timestamp"] + params_to_save
        all_values = [metadata_text, run_id] + param_values
        condig_dict = {'Parameter Names': all_names,
                       'Parameter Values': all_values}
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is saving the config settings in:\n\t" + config_save_path + "\n")
        config_df = pd.DataFrame(data=condig_dict)
        config_df.to_csv(config_save_path, index=False)

        status_pub.publish(std_msgs.msg.String("finished_saving_data"))

        if save_sim_history:
            # save simulation history
            background = set_up_image_creation()
            frames = [create_image_from_row(row, background, index, len(records_df))
                      for index, row in records_df.iterrows()]
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is creating and saving the simulation history in:\n\t" + simulation_history_save_path + "\n\n\tThis can take some minutes if the dataset is relatively large.\n")
            # for GIF:
            # imageio.mimsave(simulation_history_save_path, frames, fps=sim_history_fps)
            # for MP4:
            size = frames[0].shape
            out = cv2.VideoWriter(simulation_history_save_path, cv2.VideoWriter_fourcc(
                *'mp4v'), sim_history_fps, (size[1], size[0]))
            for frame in frames:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                out.write(frame)
            out.release()
        else:
            # to not send directly two finish signals after each other
            rospy.sleep(0.5)
    else:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Wrapper is not actually active.\n")

    status_pub.publish(std_msgs.msg.String("finished"))

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')


if __name__ == '__main__':
    try:
        data_collector()
    except rospy.ROSInterruptException:
        pass
