#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "09/2020"

"""
This node will launch the actual simulator.
It is an actual wrapper node but will not have the wrapper prefix/namespace (otherwise all launched simulator nodes will have it, too).
It will be commanded by the control master.
"""

# IMPORTS
# ---------------------------------------------
import sys
import os.path

import rospy
import rospkg
import roslaunch

from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger, TriggerResponse


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "sim_launcher"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables which will be updated in the inital setup phase:
launch_simulator = False
launch_success = None
launch_msg = ""

terminate_node = False

simulator = None

shutdown_sim_pub = None

# CODE
# ---------------------------------------------


def handle_sim_launch_request(req):
    global launch_simulator
    # Request for a simulator launch
    green_color = '\033[92m'  # green
    end_color = '\033[0m'  # indicates end of color
    string = ""
    for _ in range(80):
        string += "X"
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a simulator launch request.\n"
        + "\n\n" + green_color + string + "\n\tYou will read now also the output of the provided simulator...\n" + string + end_color + "\n\n\n")

    # Set global signal
    launch_simulator = True

    success = False
    return_msg = ""

    # Wait for success response
    rate = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown():
        if launch_success != None:
            success = launch_success
            return_msg = launch_msg
            break
        rate.sleep()
    return TriggerResponse(success=success, message=return_msg)


def handle_sim_termination_request(req):
    global simulator, terminate_node
    # Request for a simulator termination
    green_color = '\033[92m'  # green
    # red_color = '\033[91m'  # red
    yellow_color = "\033[33m"  # yellow
    # magenta_color = "\033[35m"  # magenta
    # white_background = "\033[107m"
    end_color = '\033[0m'  # indicates end of color
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a simulator termination request."
        f"\n\t{yellow_color}The shutdown can take a bit if some nodes are blocked and needs to be forced..."
        f"\n\tDo not finish it manually since the evaluation still needs to run!{end_color}\n")

    success = False
    return_msg = ""

    # terminate simulator:
    if simulator != None:
        simulator.shutdown()
        shutdown_sim_pub.publish(Empty())
        success = True
        terminate_node = True
        string = ""
        for _ in range(110):
            string += "X"
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has terminated the simulator.\n"
            + "\n\n" + green_color + string + "\n\tEnd of simulation. The evaluation nodes are sill running (if evaluation is enabled)...\n" + string + end_color + "\n\n\n")
    else:
        return_msg = "Simulator object is emtpy. Cannot terminate it."
    return TriggerResponse(success=success, message=return_msg)


def shutdown_callback(message):
    global terminate_node
    # message is of type Empty
    terminate_node = True


def sim_launcher():
    global terminate_node, simulator, shutdown_sim_pub, launch_simulator, launch_success, launch_msg

    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    rospack = rospkg.RosPack()

    shutdown_everything = False

    # Get all needed parameters
    wrapper_namespace = rospy.get_param("/wrapper_namespace")
    sim_launch_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/pkg_of_simulator_launch")
    internal_launch_path = rospy.get_param(
        "/" + wrapper_namespace + "/internal_path_to_simulator_launch")
    absolute_launch_path = rospy.get_param(
        "/" + wrapper_namespace + "/absolute_path_to_simulator_launch")
    request_sim_launch = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/request_sim_launch")
    request_sim_termination = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/request_sim_termination")
    final_shutdown_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    shutdown_sim_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/shutdown_sim_topic")
    sim_launcher_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/sim_launcher_status_topic")
    final_shutdown_request_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_request_topic")
    base_map_pkg_name = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_pkg")
    base_map_folder = rospy.get_param(
        "/" + wrapper_namespace + "/base_map_folder")
    map_name = rospy.get_param(
        "/" + wrapper_namespace + "/map_name")
    gazebo_world_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/gazebo_world_pkg")
    gazebo_world_internal_path = rospy.get_param(
        "/" + wrapper_namespace + "/gazebo_world_path")
    # Get launch arguments from the parameter server
    sim_no_of_robots = rospy.get_param(
        "/" + wrapper_namespace + "/sim_no_of_robots")
    sim_gui = rospy.get_param(
        "/" + wrapper_namespace + "/sim_gui")
    sim_seed = rospy.get_param(
        "/" + wrapper_namespace + "/sim_seed")
    sim_external_arguments = rospy.get_param(
        "/" + wrapper_namespace + "/sim_external_arguments")
    sim_starting_points = rospy.get_param(
        "/" + wrapper_namespace + "/sim_starting_points")

    # wait in the background for sim launch signal from the control master node
    rospy.Service(request_sim_launch,
                  Trigger, handle_sim_launch_request)

    # wait in the background for sim termination signal from the control master node
    rospy.Service(request_sim_termination, Trigger,
                  handle_sim_termination_request)

    # Subscribers
    shutdown_sub = rospy.Subscriber(
        final_shutdown_topic, Empty, shutdown_callback)

    # Publishers
    shutdown_pub = rospy.Publisher(
        final_shutdown_request_topic, Empty, queue_size=100)
    shutdown_sim_pub = rospy.Publisher(
        shutdown_sim_topic, Empty, queue_size=100)
    status_pub = rospy.Publisher(
        sim_launcher_status_topic, String, queue_size=100)

    # get path to base_map file (actually the yaml file which is referencing the pgm file):
    base_map_pkg = rospack.get_path(base_map_pkg_name)
    base_map_file = base_map_pkg + "/" + base_map_folder + \
        "/" + map_name + "/" + map_name + ".yaml"

    # get path to world file:
    world_pkg = rospack.get_path(gazebo_world_pkg)
    world_file = world_pkg + gazebo_world_internal_path

    # get path to the launch file
    final_launch_path = ""
    if absolute_launch_path != None and absolute_launch_path != "" and absolute_launch_path != "none" and absolute_launch_path != "None":
        # absolute path provided: take it directly
        final_launch_path = absolute_launch_path
    elif (sim_launch_pkg != None and sim_launch_pkg != "" and sim_launch_pkg != "none" and sim_launch_pkg != "None"
          and internal_launch_path != None and internal_launch_path != "" and internal_launch_path != "none" and internal_launch_path != "None"):
            # No absolute path provided, but pkg and internal path. So, we combine them:
        pkg_path = rospack.get_path(sim_launch_pkg)
        final_launch_path = pkg_path + "/" + internal_launch_path
    else:
        # Neither absolute nor pkg path is provided: not possible to get the launch file
        rospy.logerr(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': No path to the main launch file for the simulator was provided.\n"
                     + "\tCannot start simulator - Wrapper will terminate.\n"
                     + "\tWrite the path to the simulator launch file inside the evaluation master launch file.\n")
        # terminate all other nodes and then also this node
        shutdown_everything = True
        shutdown_pub.publish(Empty())

    # Check if file exists:
    if not os.path.isfile(final_launch_path):
        rospy.logerr(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': The provided path to the main launch file for the simulator is not valid (file does not exist).\n"
                     f"\tProvided: '{final_launch_path}'\n"
                     + "\tCannot start simulator - Wrapper will terminate.\n"
                     + "\tWrite the path to the simulator launch file inside the evaluation master launch file.\n")
        # terminate all other nodes and then also this node
        shutdown_everything = True
        shutdown_pub.publish(Empty())

    # check if enough arguments are there for the wanted robot number:
    if len(sim_starting_points) < sim_no_of_robots:
        rospy.logerr(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': There are not enough starting points (={str(len(sim_starting_points))}) specified in parameters.yaml for the wanted number of robots (={str(sim_no_of_robots)}).\n"
                     + "\tCannot start simulator - Wrapper will terminate.\n")
        # terminate all other nodes and then also this node
        shutdown_everything = True
        shutdown_pub.publish(Empty())

    if not shutdown_everything:
        rospy.loginfo(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready -"
                      f"\n\t...will wait continously for simulator launch request at '{request_sim_launch}'"
                      f"\n\t...will wait continously for simulator termination request at '{request_sim_termination}'"
                      f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                      f"\n\t...will publish its status to '{status_pub.resolved_name}'"
                      f"\n\t...will publish the simulator shutdown signal to '{shutdown_sim_pub.resolved_name}'"
                      f"\n\t...will launch the simulator with file:\n\t'{final_launch_path}'\n")

    # signals are done by callbacks / received requests

    # launching the simulator cannot be done by a service, but needs to be in the main thread
    # So, I need to run a while loop
    rate = rospy.Rate(1)  # Hz
    not_yet_launched = True
    while not rospy.is_shutdown() and not terminate_node and not shutdown_everything:
        if not_yet_launched:
            # signal ready:
            status_pub.publish(String("ready"))

        if launch_simulator:
            launch_simulator = False
            # launch simulator:
            if final_launch_path != "":
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                # basic arguments:
                cli_args = [final_launch_path,
                            "sim_gui:=" + str(sim_gui),
                            "sim_seed:=" + str(sim_seed),
                            "sim_no_of_robots:=" + str(sim_no_of_robots),
                            "sim_base_map_file:=" + base_map_file,
                            "sim_gazebo_world_file:=" + world_file]
                for robot_index in range(sim_no_of_robots):
                    cli_args.append("sim_r" + str(robot_index) +
                                    "_x_start:=" + str(sim_starting_points[robot_index]["x"]))
                    cli_args.append("sim_r" + str(robot_index) +
                                    "_y_start:=" + str(sim_starting_points[robot_index]["y"]))
                # if external arguments are added:
                if sim_external_arguments != "":
                    # first split string at blank spaces
                    separated_args = sim_external_arguments.split()
                    # then add each of them
                    for argument_string in separated_args:
                        cli_args.append(argument_string)
                roslaunch_args = cli_args[1:]
                roslaunch_file = [
                    (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
                simulator = roslaunch.parent.ROSLaunchParent(
                    uuid, roslaunch_file)
                simulator.start()
                launch_success = True
                not_yet_launched = True
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has started the simulator with these arguments:\n" + str(cli_args) + "\n")
            else:
                launch_success = False
                launch_msg = "Path of simulator launch file was emtpy:" + final_launch_path
        rate.sleep()

    if shutdown_everything:
        rospy.sleep(1)
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is sending termination request.\n")
        shutdown_pub.publish(Empty())

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')


if __name__ == '__main__':
    try:
        sim_launcher()
    except rospy.ROSInterruptException:
        pass
