#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "09/2020"

"""
This node will start the actual launch file (evaluation_master.launch) which then starts all other nodes, etc.
This way one node (this one) can at the end terminate the launch file and with it all linked nodes.
Afterwards only this node should exist and can terminate itself.
The final shutdown command/signal will come from the control master.
"""

# IMPORTS
# ---------------------------------------------
import sys
import os.path
from datetime import datetime

import rospy
import rospkg
import roslaunch

from std_msgs.msg import Empty


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "main_launcher"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables
evaluation = None

# CODE
# ---------------------------------------------


def shutdown_callback(message):
    # message is of type Empty
    rospy.sleep(1)
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' will terminate the main launch file.\n")
    evaluation.shutdown()

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')


def main_launcher():
    global evaluation
    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    wrapper_namespace = rospy.get_param("/wrapper_namespace")

    config_name = rospy.get_param(
        "/" + wrapper_namespace + "/config_name")

    # update run id:
    now = datetime.now()
    prefix = rospy.get_param(
        "/" + wrapper_namespace + "/run_id")
    run_id = now.strftime("%Y-%m-%d_%H-%M-%S")
    if prefix != "":
        run_id = prefix + "_" + run_id
    rospy.set_param("/" + wrapper_namespace +
                    "/run_id", run_id)

    # update file names with run id:
    recorded_data_csv = rospy.get_param(
        "/" + wrapper_namespace + "/recorded_data_csv")
    splits = recorded_data_csv.split('.')
    length_until_last_split = len(
        recorded_data_csv) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = recorded_data_csv[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/recorded_data_csv", new_file_name)

    result_file = rospy.get_param(
        "/" + wrapper_namespace + "/result_file")
    splits = result_file.split('.')
    length_until_last_split = len(
        result_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = result_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/result_file", new_file_name)

    config_save_file = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_file")
    splits = config_save_file.split('.')
    length_until_last_split = len(
        config_save_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = config_save_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/config_save_file", new_file_name)

    summary_file = rospy.get_param(
        "/" + wrapper_namespace + "/summary_file")
    splits = summary_file.split('.')
    length_until_last_split = len(
        summary_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = summary_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/summary_file", new_file_name)

    grid_file = rospy.get_param(
        "/" + wrapper_namespace + "/grid_file")
    splits = grid_file.split('.')
    length_until_last_split = len(
        grid_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = grid_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/grid_file", new_file_name)

    distribution_image = rospy.get_param(
        "/" + wrapper_namespace + "/distribution_image")
    splits = distribution_image.split('.')
    length_until_last_split = len(
        distribution_image) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = distribution_image[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/distribution_image", new_file_name)

    evolution_file = rospy.get_param(
        "/" + wrapper_namespace + "/evolution_file")
    splits = evolution_file.split('.')
    length_until_last_split = len(
        evolution_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = evolution_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/evolution_file", new_file_name)

    simulation_history_file = rospy.get_param(
        "/" + wrapper_namespace + "/simulation_history_file")
    splits = simulation_history_file.split('.')
    length_until_last_split = len(
        simulation_history_file) - len(splits[-1]) - 1  # -1 because of the dot
    new_file_name = simulation_history_file[:length_until_last_split] + \
        "_" + run_id + "." + splits[-1]
    rospy.set_param("/" + wrapper_namespace +
                    "/simulation_history_file", new_file_name)

    # Get all needed parameters
    launchers_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/launchers_pkg")
    config_path = rospy.get_param(
        "/" + wrapper_namespace + "/config_path")
    evaluation_master_path = rospy.get_param(
        "/" + wrapper_namespace + "/evaluation_master_path")
    final_shutdown_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    base_map_pkg_name = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_pkg")
    base_map_folder = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_folder")
    map_name = rospy.get_param(
        "/" + wrapper_namespace + "/map_name")
    simulator_namespace = rospy.get_param(
        "/" + wrapper_namespace + "/simulator_namespace")
    active_wrapper = rospy.get_param(
        "/" + wrapper_namespace + "/active_wrapper")
    active_wrapper_string = "1"
    if not active_wrapper:
        active_wrapper_string = "0"

    # get path to the config file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(launchers_pkg)
    abs_config_path = pkg_path + config_path

    green_color = '\033[92m'  # green
    end_color = '\033[0m'  # indicates end of color
    rospy.loginfo(
        "*** WRAPPER MESSAGE ***\n\n\tAll wrapper parameters are set up in the ROS parameter server.\n\tThe configuration name is: '" + green_color + config_name + end_color + "'\n\tThey are loaded from: '" + green_color + abs_config_path + end_color + "'\n")

    # Subscribers
    shutdown_sub = rospy.Subscriber(
        final_shutdown_topic, Empty, shutdown_callback)

    # get path to the launch file
    launch_path = pkg_path + evaluation_master_path

    # get path to base_map file:
    base_map_pkg = rospack.get_path(base_map_pkg_name)
    base_map_file = base_map_pkg + "/" + base_map_folder + \
        "/" + map_name + "/" + map_name + ".yaml"

    rospy.loginfo(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
                  f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                  f"\n\t...will launch the evaluation with file:\n\t'{launch_path}'\n")

    # launch everything else with the evaluation launch file:
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = [launch_path,
                "wrapper_namespace:=" + wrapper_namespace,
                "simulator_namespace:=" + simulator_namespace,
                "base_map_file:=" + base_map_file,
                "active_wrapper:=" + active_wrapper_string]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [
        (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    evaluation = roslaunch.parent.ROSLaunchParent(
        uuid, roslaunch_file)
    evaluation.start()
    if active_wrapper:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has started the complete evaluation process.\n")
    else:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has started the control master and the simulator launcher.\n")

    # wait/spin for the shutdown signal (received and executed in the callback methods)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_launcher()
    except rospy.ROSInterruptException:
        pass
