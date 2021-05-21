#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "09/2020"

"""
This is the control node of the evaluation framework/wrapper.
It checks if the wrapper is correctly set up and as soon as this is the case,
the node will signals the sim_launcher to launch the provided main launch 
file for the simulator.
It will also keep control over the time and termination of the other nodes.
"""

# IMPORTS
# ---------------------------------------------
import sys
import os
import subprocess
import csv

import rospy
import rospkg

from std_msgs.msg import Header, String, Empty, Int32
from std_srvs.srv import Trigger


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "control_master"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

# Global variables:
sim_launcher_status = "not_ready"
map_provider_status = "not_ready"
global_dirt_generator_status = "not_ready"
world_generator_status = "not_ready"
data_collector_status = "not_ready"
evaluator_status = "not_ready"

active_wrapper = True

evaluation_finished = False

sim_setup_finished = False

final_shutdown_request = False


# CODE
# ---------------------------------------------


def signal_sim_launch(service_name):
    rospy.wait_for_service(service_name)
    try:
        request = rospy.ServiceProxy(service_name, Trigger)
        response = request()  # Trigger service does not need any input
        return (response.success, response.message)
    except rospy.ServiceException as e:
        rospy.logerr(
            "Service call failed (requesting simulator launch): %s", e)


def signal_sim_termination(service_name):
    rospy.wait_for_service(service_name)
    try:
        request = rospy.ServiceProxy(service_name, Trigger)
        response = request()  # Trigger service does not need any input
        return (response.success, response.message)
    except rospy.ServiceException as e:
        rospy.logerr(
            "Service call failed (requesting simulator termiantion): %s", e)


def final_shutdown_request_callback(message):
    global final_shutdown_request
    # message is of type Empty and signals the final shutdown
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a global termination request from another node.\n")
    final_shutdown_request = True


def evaluation_finished_callback(message):
    global evaluation_finished
    # message is of type Empty and signals the end of the evaluation
    evaluation_finished = True


def sim_launcher_callback(message):
    global sim_launcher_status
    # message is of type String
    sim_launcher_status = message.data


def map_provider_callback(message):
    global map_provider_status
    # message is of type String
    map_provider_status = message.data


def global_dirt_generator_callback(message):
    global global_dirt_generator_status
    # message is of type String
    global_dirt_generator_status = message.data


def world_generator_callback(message):
    global world_generator_status
    # message is of type String
    world_generator_status = message.data


def data_collector_callback(message):
    global data_collector_status
    # message is of type String
    data_collector_status = message.data


def evaluator_callback(message):
    global evaluator_status
    # message is of type String
    evaluator_status = message.data


def sim_setup_finished_callback(message):
    global sim_setup_finished
    # message is of type Empty and means that the sim setup is finished
    sim_setup_finished = True


def check_all_ready():
    # main_launcher will be started and ready always before control_master
    # --> no need to check him
    # map_server cannot be checked since it is not my own node,
    # as soons as map_provider is ready, it means that also map_server is publishing everything needed
    if active_wrapper:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}':\n\tSim launcher: " + sim_launcher_status + "\n\tMap provider: " +
            map_provider_status + "\n\tDirt generator: " + global_dirt_generator_status + "\n\tWorld generator: " + world_generator_status + "\n\tData collector: " + data_collector_status + "\n\tEvaluator: " + evaluator_status + "\n")
        return (sim_launcher_status == "ready"
                and map_provider_status == "ready"
                and global_dirt_generator_status == "ready"
                and world_generator_status == "ready"
                and data_collector_status == "ready"
                and evaluator_status == "ready")
    else:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}':\n\tSim launcher: " + sim_launcher_status + "\n")
        return (sim_launcher_status == "ready")


def control_master():
    global active_wrapper, sim_setup_finished
    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    process_start_time = rospy.get_time()

    # Get all needed parameters
    wrapper_namespace = rospy.get_param("/wrapper_namespace")
    # since this node is inside the wrapper namespace, we could also get it
    # with rospy.get_namespace(), but then we would get "/wrapper/" instead of "wrapper"
    request_sim_launch = rospy.get_param(
        "/" + wrapper_namespace + "/request_sim_launch")
    request_sim_launch_absolute = "/" + wrapper_namespace + "/" + request_sim_launch
    request_sim_termination = rospy.get_param(
        "/" + wrapper_namespace + "/request_sim_termination")
    request_sim_termination_absolute = "/" + \
        wrapper_namespace + "/" + request_sim_termination
    final_shutdown_topic = rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    start_dirt_gen_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/start_dirt_gen_topic")
    stop_dirt_gen_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/stop_dirt_gen_topic")
    stop_data_collector_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/stop_data_collector_topic")
    start_evaluation_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/start_evaluation_topic")
    evaluation_finished_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/evaluation_finished_topic")

    sim_launcher_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/sim_launcher_status_topic")
    map_provider_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/map_provider_status_topic")
    global_dirt_generator_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/global_dirt_generator_status_topic")
    world_generator_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/world_generator_status_topic")
    data_collector_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/data_collector_status_topic")
    evaluator_status_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/evaluator_status_topic")
    final_shutdown_request_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_request_topic")

    sim_setup_end_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/sim_setup_end_topic")

    output_pkg = rospy.get_param(
        "/" + wrapper_namespace + "/output_pkg")
    recorded_data_folder = rospy.get_param(
        "/" + wrapper_namespace + "/recorded_data_folder")
    recorded_data_csv = rospy.get_param(
        "/" + wrapper_namespace + "/recorded_data_csv")
    results_folder = rospy.get_param(
        "/" + wrapper_namespace + "/results_folder")
    result_file = rospy.get_param(
        "/" + wrapper_namespace + "/result_file")
    summary_folder = rospy.get_param(
        "/" + wrapper_namespace + "/summary_folder")
    summary_file = rospy.get_param(
        "/" + wrapper_namespace + "/summary_file")
    config_save_folder = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_folder")
    config_save_file = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_file")

    simulation_time = rospy.get_param(
        "/" + wrapper_namespace + "/simulation_time")
    simulation_clock_topic = rospy.get_param(
        "/" + wrapper_namespace + "/simulation_clock_topic")

    active_wrapper = rospy.get_param(
        "/" + wrapper_namespace + "/active_wrapper")

    sim_external_command = rospy.get_param(
        "/" + wrapper_namespace + "/sim_external_command")

    time_update_interval = rospy.get_param(
        "/" + wrapper_namespace + "/time_update_interval")

    # Subscribers
    sim_launcher_sub = rospy.Subscriber(
        sim_launcher_status_topic, String, sim_launcher_callback)
    shutdown_request_sub = rospy.Subscriber(
        final_shutdown_request_topic, Empty, final_shutdown_request_callback)
    if active_wrapper:
        map_provider_sub = rospy.Subscriber(
            map_provider_status_topic, String, map_provider_callback)
        global_dirt_generator_sub = rospy.Subscriber(
            global_dirt_generator_status_topic, String, global_dirt_generator_callback)
        world_generator_sub = rospy.Subscriber(
            world_generator_status_topic, String, world_generator_callback)
        data_collector_sub = rospy.Subscriber(
            data_collector_status_topic, String, data_collector_callback)
        evaluator_sub = rospy.Subscriber(
            evaluator_status_topic, String, evaluator_callback)
        evaluation_finished_sub = rospy.Subscriber(
            evaluation_finished_topic, Empty, evaluation_finished_callback)
        sim_setup_end_sub = rospy.Subscriber(
            sim_setup_end_topic, Empty, sim_setup_finished_callback)

    # Publishers
    shutdown_pub = rospy.Publisher(
        final_shutdown_topic, Empty, queue_size=100)
    if active_wrapper:
        stop_gen_pub = rospy.Publisher(
            stop_dirt_gen_topic, Empty, queue_size=100)
        stop_collector_pub = rospy.Publisher(
            stop_data_collector_topic, Empty, queue_size=100)
        simulation_clock_pub = rospy.Publisher(
            simulation_clock_topic, Int32, queue_size=100)
        start_evaluation_pub = rospy.Publisher(
            start_evaluation_topic, Empty, queue_size=100)
        start_dirt_gen_pub = rospy.Publisher(
            start_dirt_gen_topic, Empty, queue_size=100)

    if active_wrapper:
        # get path to the output files
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(output_pkg)
        recording_path = pkg_path + "/" + recorded_data_folder + \
            "/" + recorded_data_csv
        result_path = pkg_path + "/" + results_folder + \
            "/" + result_file
        summary_path = pkg_path + "/" + summary_folder + \
            "/" + summary_file
        config_save_path = pkg_path + "/" + config_save_folder + \
            "/" + config_save_file

        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
            f"\n\t...will request simulator launch at '{request_sim_launch_absolute}' (service)"
            f"\n\t...will request simulator termiantion at '{request_sim_termination_absolute}' (service)"
            f"\n\t...will listen continously to '{shutdown_request_sub.resolved_name}' for the final termination request sent by another nodes"
            f"\n\t...will listen continously to '{sim_setup_end_sub.resolved_name}' for the end signal of the sim setup"
            f"\n\t...will listen continously to '{sim_launcher_sub.resolved_name}' for status of sim_launcher"
            f"\n\t...will listen continously to '{map_provider_sub.resolved_name}' for status of map_provider"
            f"\n\t...will listen continously to '{global_dirt_generator_sub.resolved_name}' for status of global_dirt_generator"
            f"\n\t...will listen continously to '{world_generator_sub.resolved_name}' for status of world_generator"
            f"\n\t...will listen continously to '{data_collector_sub.resolved_name}' for status of data_collector"
            f"\n\t...will listen continously to '{evaluator_sub.resolved_name}' for status of evaluator"
            f"\n\t...will listen continously to '{evaluation_finished_sub.resolved_name}' for the end of the evaluation process"
            f"\n\t...will publish the start of dirt generation to '{start_dirt_gen_pub.resolved_name}'"
            f"\n\t...will publish the end of dirt generation to '{stop_gen_pub.resolved_name}'"
            f"\n\t...will publish the simulation clock to '{simulation_clock_pub.resolved_name}'"
            f"\n\t...will publish evaluation start signal to '{start_evaluation_pub.resolved_name}'"
            f"\n\t...will publish global shutdown signal to '{shutdown_pub.resolved_name}'\n")
    else:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
            f"\n\t...will request simulator launch at '{request_sim_launch_absolute}' (service)"
            f"\n\t...will request simulator termiantion at '{request_sim_termination_absolute}' (service)"
            f"\n\t...will listen continously to '{shutdown_request_sub.resolved_name}' for the final termination request sent by another nodes"
            f"\n\t...will listen continously to '{sim_launcher_sub.resolved_name}' for status of sim_launcher"
            f"\n\t...will publish global shutdown signal to '{shutdown_pub.resolved_name}'\n")

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Waiting for all needed wrapper nodes to be ready to launch simulator.\n")
    rate = rospy.Rate(1)  # Hz
    while not rospy.is_shutdown() and not final_shutdown_request:
        # wait for start signals from all other wrapper nodes
        if check_all_ready():
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': All wrapper nodes have sent the 'ready' flag.\n")
            break
        rate.sleep()

    # and then signals the sim_launcher to launch the simulator
    if not final_shutdown_request:
        successful, info_msg = signal_sim_launch(request_sim_launch)
        if successful and not final_shutdown_request:
            launch_simulation_timestamp = rospy.get_time()
            wrapper_setup_time = launch_simulation_timestamp - process_start_time
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Simulator was started.\n")

            # Start external command if it was added to the parameters:
            if sim_external_command != "" and not final_shutdown_request:
                command = "gnome-terminal --tab -- bash -c '" + sim_external_command + \
                    "; exec bash'"  # Leave the tab open after finishing (with exec bash at the end)
                subprocess.Popen(command, shell=True)

            # for the case if wrapper is not active and simulation will be counted directly from the beginning on:
            active_simulation_timestamp = rospy.get_time()
            # wait until sim setup is finished
            rate = rospy.Rate(2)  # Hz
            while not rospy.is_shutdown() and not final_shutdown_request and active_wrapper:
                if sim_setup_finished:
                    active_simulation_timestamp = rospy.get_time()
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Simulator setup is finished. Robot(s) can move.\n")
                    # start dirt generation
                    start_dirt_gen_pub.publish(Empty())
                    break
                rate.sleep()

            # how much the generation should end before the simulation ends:
            gen_delay = 3  # s
            end_of_sim = active_simulation_timestamp + simulation_time
            next_time_update = active_simulation_timestamp + time_update_interval
            gen_stopped = False
            # '\033[92m' = green, '\033[93m' = yellow , '\033[91m' = red
            time_color = '\033[92m'
            end_color = '\033[0m'  # indicates end of color
            simulation_interval = 1  # s
            rate = rospy.Rate(1 / simulation_interval)  # Hz
            while not rospy.is_shutdown() and rospy.get_time() <= end_of_sim and not final_shutdown_request:
                current_time = rospy.get_time()
                # wait until simulator termination/shutdown should be executed
                if active_wrapper:
                    # and publish simulation time
                    simulation_clock_pub.publish(
                        Int32(data=int(round(current_time - active_simulation_timestamp))))

                if active_wrapper and not gen_stopped and current_time >= (end_of_sim - gen_delay) and not final_shutdown_request:
                    # signal dirt generation stop (if it is not already stopped) a bit earlier
                    stop_gen_pub.publish(Empty())
                    gen_stopped = True

                if current_time >= next_time_update and not final_shutdown_request:
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t{time_color}Elapsed simulation time: {str(round(current_time - active_simulation_timestamp))} s - Remaining simulation time: {str(round(end_of_sim - current_time))} s{end_color}\n")
                    next_time_update += time_update_interval

                rate.sleep()

            sim_setup_time = active_simulation_timestamp - launch_simulation_timestamp
            sim_active_time = rospy.get_time() - active_simulation_timestamp
            start_finishing = rospy.get_time()

            # signal simulation shutdown and also ending of data collection
            if active_wrapper and not final_shutdown_request:
                stop_collector_pub.publish(Empty())
            successful, info_msg = signal_sim_termination(
                request_sim_termination)
            if successful:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Simulator was terminated.\n")
            else:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Simulator could not be terminated.\n\t{info_msg}\n")

            Q_max = 0.0
            Q_avg = 0.0
            evaluation_time = 0.0
            if active_wrapper:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Waiting until data_collector has finished.\n")
                rate = rospy.Rate(1)  # Hz
                while not rospy.is_shutdown() and not final_shutdown_request:
                    # wait until data_collector has finished saving data
                    if data_collector_status == "finished_saving_data" or data_collector_status == "finished":
                        rospy.loginfo(
                            f"*** WRAPPER MESSAGE ***\n\n\t{time_color}'{rospy.get_caller_id()}': data_collector has finished saving the recorded data. You can find the recordings in:\n\t{recording_path}{end_color}\n")
                        rospy.sleep(0.1)
                        # start evaluator
                        start_evaluation_pub.publish(Empty())
                        evaluation_start_time = rospy.get_time()
                        rospy.loginfo(
                            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Evaluation was started...\n")
                        break
                    rate.sleep()

                rate = rospy.Rate(1)  # Hz
                while not rospy.is_shutdown() and not final_shutdown_request:
                    # wait until evaluation is finished
                    if evaluation_finished:
                        Q_splits = evaluator_status.split("-")
                        for split in Q_splits:
                            max_indicator = "Q_max:"
                            avg_indicator = "Q_avg:"
                            if split.startswith(max_indicator):
                                Q_max = float(split[len(max_indicator):])
                            if split.startswith(avg_indicator):
                                Q_avg = float(split[len(avg_indicator):])

                        evaluation_time = rospy.get_time() - evaluation_start_time
                        rospy.loginfo(
                            f"*** WRAPPER MESSAGE ***\n\n\t{time_color}'{rospy.get_caller_id()}': Evaluation was finished. If everything went well, you can find the result in:\n\t{result_path}\n\tand the whole summary in:\n\t{summary_path}{end_color}\n")
                        break
                    rate.sleep()

                if data_collector_status != "finished":
                    rospy.loginfo(
                        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Waiting for the final finish signal from data_collector. Can take a while due to gif creation for simulation history (if enabled).\n")
                    rate = rospy.Rate(1)  # Hz
                    while not rospy.is_shutdown() and not final_shutdown_request:
                        # wait until data_collector has completely finished (with saving data it is already finished, but it can be still in simulation history gif creation)
                        if data_collector_status == "finished":
                            break
                        rate.sleep()

                green_color = '\033[92m'  # green
                end_color = '\033[0m'  # indicates end of color
                string = ""
                for _ in range(80):
                    string += "X"
                rospy.loginfo(
                    "\n\n" + green_color + string + "\n\tThe evaluation process is finished\n" + string + end_color + "\n\n\n")
            else:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Since the active wrapper flag was not set, no data was recorded and thus no evaluation can be executed.\n\tThe simulation has ended, which is why the program will shutdown...\n")

            final_finishing_time = rospy.get_time() - start_finishing
            process_time = rospy.get_time() - process_start_time
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t{time_color}The final times:\n\t- Initial wrapper setup time: {str(round(wrapper_setup_time))} s\n\t- Initial simulation setup time: {str(round(sim_setup_time))} s\n\t- Counted simulation time: {str(round(sim_active_time))} s\n\t- Evaluation time: {str(round(evaluation_time))} s\n\t- Finishing time (incl. evaluation): {str(round(final_finishing_time))} s\n\t- Whole process time: {str(round(process_time))} s{end_color}\n")
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t{time_color}Max. Q-Score:\t{str(round(Q_max, 2))}\n\tAvg. Q-Score:\t{str(round(Q_avg, 2))}{end_color}\n\tEverything else: check out the output files with the current run ID...\n")
            if active_wrapper:
                green_color = '\033[92m'  # green
                end_color = '\033[0m'  # indicates end of color
                run_id = rospy.get_param(
                    "/" + wrapper_namespace + "/run_id")
                print("\nID of this run (for the output files):\t" + green_color +
                      str(run_id) + end_color + "\n")
            if not final_shutdown_request and active_wrapper:
                extra = [["XXXXXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"],
                         ["Final times:", "Times in seconds:"],
                         ["Initial wrapper setup time", wrapper_setup_time],
                         ["Initial simulation setup time", sim_setup_time],
                         ["Counted simulation time", sim_active_time],
                         ["Evaluation time", evaluation_time],
                         ["Finishing time (incl. evaluation)",
                          final_finishing_time],
                         ["Whole process time", process_time],
                         ["XXXXXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"],
                         ["Max. Q score", Q_max],
                         ["Avg. Q score", Q_avg]]
                # write extra in config file:
                in_file = open(config_save_path, "r")
                csv_list = list(csv.reader(in_file))
                in_file.close()
                csv_list.extend(extra)
                out_file = open(config_save_path, "w")
                writer = csv.writer(out_file)
                writer.writerows(csv_list)
                out_file.close()

                # write extra in summary file:
                in_file = open(summary_path, "r")
                csv_list = list(csv.reader(in_file))
                in_file.close()
                out_file = open(summary_path, "w")
                writer = csv.writer(out_file)
                for row in csv_list:
                    if len(extra) > 0 and row[0] == "" and row[1] == "":
                        row[0] = extra[0][0]
                        row[1] = extra[0][1]
                        extra.pop(0)
                    writer.writerow(row)
                # if some extra is not yet added:
                for remaining_entry in extra:
                    writer.writerow(remaining_entry)
                out_file.close()
        else:
            if not final_shutdown_request:
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Simulator could not be started.\n\t{info_msg}\n")

    # terminate everything:
    shutdown_pub.publish(Empty())
    rospy.sleep(5)
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.logwarn(
        "\n\n\n\tThis is the end of the evaluation. Currently, the last nodes should be killed...\n\tIf this is still not successful, the rosmaster will get killed \n\tor you can manually terminate the program (Ctrl+C).\n\n")
    rospy.signal_shutdown('Intentional shutdown')  # actually not needed
    # the next lines should not be needed (the inital launch file should terminate all nodes)
    os.system("rosnode kill -a")
    rospy.sleep(2)
    os.system("killall -9 rosmaster")


if __name__ == '__main__':
    try:
        control_master()
    except rospy.ROSInterruptException:
        pass
