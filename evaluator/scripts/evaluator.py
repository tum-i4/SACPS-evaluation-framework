#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "12/2020"

"""
This node will take the collected/recorded data from the simulator
and calculate the scores for different goals/metrics based on them.
In the end, the final scores will be published.
"""

# IMPORTS
# ---------------------------------------------
import sys
import os.path

import numpy as np
import pandas as pd
from pydoc import locate
from copy import deepcopy
import matplotlib.pyplot as plt

import score_functions

import rospy
import rospkg

from std_msgs.msg import Empty, String


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------
NODE_NAME = "evaluator"
# most global parameters are set up by the ROS parameter server (from parameters.yaml in launchers pkg)

FINAL_SCORE_NAME = "Q-score"
FINAL_SCORE_NAME_P = "Q-score_p"

shutdown = False
start_evaluation = False

# CODE
# ---------------------------------------------


def shutdown_callback(message):
    global shutdown
    # message is of type Empty
    shutdown = True


def start_evaluation_callback(message):
    global start_evaluation
    # message is of type Empty and signals/requests the start of the evaluation
    start_evaluation = True
    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has received a start signal.\n\tNode will start evaluating the simulator based on the recorded data.\n")


def evaluator():
    # Node init
    rospy.init_node(NODE_NAME, anonymous=True)

    # Get all needed parameters
    wrapper_namespace = rospy.get_param("/wrapper_namespace")
    active_wrapper = rospy.get_param(
        "/" + wrapper_namespace + "/active_wrapper")
    final_shutdown_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/final_shutdown_topic")
    evaluator_status_topic = rospy.get_param(
        "/" + wrapper_namespace + "/evaluator_status_topic")
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
    start_evaluation_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/start_evaluation_topic")
    evaluation_finished_topic = "/" + wrapper_namespace + "/" + rospy.get_param(
        "/" + wrapper_namespace + "/evaluation_finished_topic")
    mandatory_data_output = rospy.get_param(
        "/" + wrapper_namespace + "/mandatory_data_output")
    score_calculation = rospy.get_param(
        "/" + wrapper_namespace + "/score_calculation")
    score_calculation_order = rospy.get_param(
        "/" + wrapper_namespace + "/score_calculation_order")
    time_name = rospy.get_param("/" + wrapper_namespace + "/time_name")
    print_row_index = rospy.get_param(
        "/" + wrapper_namespace + "/print_row_index")
    summary_folder = rospy.get_param(
        "/" + wrapper_namespace + "/summary_folder")
    summary_file = rospy.get_param(
        "/" + wrapper_namespace + "/summary_file")
    config_save_folder = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_folder")
    config_save_file = rospy.get_param(
        "/" + wrapper_namespace + "/config_save_file")
    evolution_folder = rospy.get_param(
        "/" + wrapper_namespace + "/evolution_folder")
    evolution_file = rospy.get_param(
        "/" + wrapper_namespace + "/evolution_file")

    if active_wrapper:
        # Subscribers
        shutdown_sub = rospy.Subscriber(
            final_shutdown_topic, Empty, shutdown_callback)
        start_evaluation_sub = rospy.Subscriber(start_evaluation_topic,
                                                Empty, start_evaluation_callback)

        # Publishers
        status_pub = rospy.Publisher(
            evaluator_status_topic, String, queue_size=100)
        finisher_pub = rospy.Publisher(
            evaluation_finished_topic, Empty, queue_size=100)

        # get path to the output files
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(output_pkg)
        records_path = pkg_path + "/" + recorded_data_folder + \
            "/" + recorded_data_csv
        result_path = pkg_path + "/" + results_folder + \
            "/" + result_file
        summary_path = pkg_path + "/" + summary_folder + \
            "/" + summary_file
        config_save_path = pkg_path + "/" + config_save_folder + \
            "/" + config_save_file
        evolution_save_path = pkg_path + "/" + evolution_folder + \
            "/" + evolution_file

        mandatory_columns = [
            column_name for column_name in mandatory_data_output.keys()]
        mandatory_columns.append(time_name)

        possible_functions = score_functions.get_function_list()

        all_results = []

        goal_count = len(score_calculation)
        if len(score_calculation_order) > goal_count:
            rospy.logerr("\nThe number of keys in parameter 'score_calculation_order' is higher then the actual keys in 'score_calculation'.\n"
                         + "It is now very likely that the creation of the result table will throw errors!\n")
            # if it is less, it does not matter
        goals = ["" for _ in range(goal_count)]
        remaining_goal_keys = []
        for key in score_calculation:
            if key in score_calculation_order:
                order_index = score_calculation_order.index(key)
                if order_index >= goal_count:
                    rospy.logerr("\nThe key '" + key + "' has a higher order index in 'score_calculation_order' than there are keys in 'score_calculation'.\n"
                                 + "If there are placeholders left in the end, it will be added there, but otherwise the key will be ignored!\n")
                    remaining_goal_keys.append(key)
                else:
                    goals[order_index] = key
            else:
                remaining_goal_keys.append(key)
        for key in remaining_goal_keys:
            # replace the remaining placeholders "" with the remaining keys
            if "" in goals:
                goals[goals.index("")] = key
            else:
                goals.append(key)
        goals.append(FINAL_SCORE_NAME)
        goals.append(FINAL_SCORE_NAME_P)

        rospy.loginfo(f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is ready - "
                      f"\n\t...will publish its status to '{status_pub.resolved_name}'"
                      f"\n\t...will publish the evaluation end to '{finisher_pub.resolved_name}'"
                      f"\n\t...will listen continously to '{shutdown_sub.resolved_name}' for shutdown signal"
                      f"\n\t...will listen continously to '{start_evaluation_sub.resolved_name}' for evaluation start signal"
                      f"\n\t...will take later this csv file as basis:\n\t{records_path}"
                      f"\n\t...will save the calculated score to:\n\t{result_path}"
                      f"\n\t...will save the result summary to:\n\t{summary_path}\n")

        # Publish ready and wait for start signal
        rate = rospy.Rate(1)  # Hz
        while not rospy.is_shutdown() and not shutdown:
            status_pub.publish(String("ready"))
            if start_evaluation:
                break
            rate.sleep()

        if not shutdown:
            # Start signal received --> csv file should have been created:
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is starting the evaluation based on:\n\t{records_path}\n")
            records_dataframe = pd.read_csv(
                records_path, usecols=mandatory_columns)
        break_flag = False
        if not shutdown:
            # calculate for each row all single goal scores and then the final score for this row
            for row in records_dataframe.iterrows():
                new_result_row = {}
                row_values = row[1]
                mission_goal_scores = []
                adaptation_goal_scores = []
                # for this row, go through all single goals and calculate their scores
                for goal, description in score_calculation.items():
                    function = possible_functions[description["function_index"]]
                    variable_dict = deepcopy(description["variables"])
                    for key, var in variable_dict.items():
                        if isinstance(var, str):
                            # if string, it is either a variable name or a parameter name:
                            parameter_indicator = "parameter:"
                            if var.startswith(parameter_indicator):
                                # remove the indicator and get the value of the global parameter
                                parameter_name = var[len(parameter_indicator):]
                                full_parameter_name = "/" + wrapper_namespace + "/" + parameter_name
                                if rospy.has_param(full_parameter_name):
                                    variable_dict[key] = rospy.get_param(
                                        full_parameter_name)
                                else:
                                    rospy.logerr("The parameter name '" + full_parameter_name +
                                                 "' could not be found. Evaluation is not possible!")
                                    break_flag = True
                                    break
                            else:
                                # otherwise it is directly a variable name of the recorded outputs
                                # -> get the current row value of this variable
                                variable_dict[key] = row_values[var]
                    if break_flag:
                        break
                    score = function(**variable_dict)
                    if description["type"] == "adaptation_goal":
                        adaptation_goal_scores.append(
                            score * description["weight"])
                    else:
                        mission_goal_scores.append(score)
                    new_result_row[goal] = score
                if break_flag:
                    break
                # The score is the product of all single business/mission goal scores multiplied with the weighted sum of all single adaptation goals scores.
                final_score = np.prod(mission_goal_scores) * \
                    sum(adaptation_goal_scores)
                new_result_row[FINAL_SCORE_NAME] = final_score
                new_result_row[FINAL_SCORE_NAME_P] = round(
                    final_score * 100.0, 2)
                all_results.append(new_result_row)

        if not break_flag and not shutdown:
            # create sub header
            sub_headers = []
            for goal in goals:
                if goal == FINAL_SCORE_NAME:
                    sub_headers.append("final_score")
                elif goal == FINAL_SCORE_NAME_P:
                    sub_headers.append("in_percentage")
                else:
                    sub_headers.append(
                        score_calculation[goal]["type"] + "_score")

            # write result in result_path
            all_results_df = pd.DataFrame(all_results, columns=goals)
            pure_all_results_df = deepcopy(all_results_df)
            # get max Q and avg Q
            Q_list = all_results_df[FINAL_SCORE_NAME]
            # the first entries can sometimes strange, e.g. directly 100% because no dirt is spawned yet
            Q_list = Q_list[3:]
            max_Q = max(Q_list)
            avg_Q = 0.0 if len(Q_list) == 0 else sum(Q_list) / len(Q_list)
            status_pub.publish(
                String("Q_max:" + str(max_Q) + "-Q_avg:" + str(avg_Q)))
            all_results_df.columns = pd.MultiIndex.from_tuples(
                zip(goals, sub_headers))
            all_results_df.to_csv(result_path, index=print_row_index)
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has stored the results/scores in :\n\t{result_path}\n")

            # create score evolution diagram
            if "theoretical_elapsed_time" in records_dataframe.keys():
                plt.rcParams['legend.title_fontsize'] = 18
                plt.figure(figsize=(25, 10))
                plt.title("Q score evolution over time",
                          fontsize=24, fontweight="bold")
                plt.ylabel("Q score", fontsize=20)
                plt.yticks(fontsize=16)
                plt.xlabel("time [sec]", fontsize=20)
                plt.xticks(fontsize=16)
                times = records_dataframe[time_name].tolist()
                goals.remove(FINAL_SCORE_NAME_P)
                colors = ["tab:blue", "tab:green", "tab:orange", "tab:purple", "tab:pink", "tab:cyan", "tab:olive",
                          "tab:brown", "tab:gray"]  # should contain red
                for index, goal in enumerate(goals):
                    score_list = pure_all_results_df[goal].tolist()
                    if goal == FINAL_SCORE_NAME:
                        plt.plot(times, score_list, label=goal,
                                 color="tab:red", linewidth=5.0)
                    else:
                        goal_linestyle = "dotted"
                        if score_calculation[goal]["type"] == "adaptation_goal":
                            goal_linestyle = "dashed"
                        if index < len(colors):
                            plt.plot(times, score_list, label=goal,
                                     color=colors[index], linestyle=goal_linestyle, linewidth=4.0)
                        else:
                            plt.plot(times, score_list, label=goal,
                                     linestyle=goal_linestyle, linewidth=4.0)
                plt.legend(title="Single goals and total Q (in red):",
                           fontsize=20, fancybox=True, bbox_to_anchor=(1.01, 1), loc='upper left')
                plt.tight_layout()

                plt.savefig(evolution_save_path)
                rospy.loginfo(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has stored the score evolution diagram in :\n\t{evolution_save_path}\n")
            else:
                rospy.logerr(
                    f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': 'theoretical_elapsed_time' cannot be found as recorded data. Cannot create score evolution diagram.\n")

            # create summary
            empty_sub_headers = "**************"
            placeholder_1_string = "XXX"
            placeholder_2_string = "X"

            config_dataframe = pd.read_csv(config_save_path)
            config_sub_headers = [empty_sub_headers for _ in range(
                len(config_dataframe.columns))]
            config_dataframe.columns = pd.MultiIndex.from_tuples(
                zip(config_dataframe.columns, config_sub_headers))

            records_sub_headers = [empty_sub_headers for _ in range(
                len(records_dataframe.columns))]
            records_dataframe.columns = pd.MultiIndex.from_tuples(
                zip(records_dataframe.columns, records_sub_headers))

            placeholder_1 = [
                placeholder_1_string for _ in range(len(all_results_df))]
            placeholder_1_df = pd.DataFrame(
                placeholder_1, columns=[placeholder_1_string])
            placeholder_1_df.columns = pd.MultiIndex.from_tuples(
                zip(placeholder_1_df.columns, [placeholder_1_string]))

            placeholder_2 = [
                placeholder_2_string for _ in range(len(all_results_df))]
            placeholder_2_df = pd.DataFrame(
                placeholder_2, columns=[placeholder_2_string])
            placeholder_2_df.columns = pd.MultiIndex.from_tuples(
                zip(placeholder_2_df.columns, [placeholder_2_string]))

            # First the parameters, then the results and at the end the records
            summary_list = [config_dataframe, placeholder_1_df,
                            all_results_df, placeholder_2_df, records_dataframe]
            keys = ["Parameter Configuration", placeholder_1_string,
                    "Results", placeholder_2_string, "Recordings"]
            summary_df = pd.concat(summary_list, axis=1, keys=keys)

            # write summary to summary_path
            summary_df.to_csv(summary_path, index=False)
            rospy.loginfo(
                f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has stored the summary in :\n\t{summary_path}\n")

        # Evaluation is finished
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' has finished the evaluation.\n")
        finisher_pub.publish(Empty())
    else:
        rospy.loginfo(
            f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}': Wrapper is not actually active.\n")

    rospy.loginfo(
        f"*** WRAPPER MESSAGE ***\n\n\t'{rospy.get_caller_id()}' is terminating.\n")
    rospy.signal_shutdown('Intentional shutdown')


if __name__ == '__main__':
    try:
        evaluator()
    except rospy.ROSInterruptException:
        pass
