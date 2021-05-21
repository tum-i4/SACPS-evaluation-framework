#!/usr/bin/env python3

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "12/2020"

"""
Here, all functions are defined which can be used for score
calculation inside evaluator.py.
It is possible to add more functions.
Old functions should preferable not be modified.
"""

# FUNCTION LIST GETTER
# ---------------------------------------------


def get_function_list():
    # all function from below needs to be added here in the correct order
    # so that they can be identified with their index
    function_list = [function_0, function_1, function_2,
                     function_3, function_4, function_5,
                     function_6]
    # --> in score_calculation (in parameters.yaml) "function_index: 0"
    # will let the evaluator call function_0, "function_index: 1" function_1, etc.
    return function_list

# SCORE FUNCTIONS
# ---------------------------------------------


def function_0(portion_number, total_amount):
    # the higher the portion is in comparison to the total amount, the better it is
    # if it is everything, output will be 1
    # if it is nothing, output will be 0
    if portion_number is None or total_amount is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif total_amount == 0:
        return 1.0  # it is not possible to have a portion of nothing
    elif portion_number < 0 or total_amount < 0 or portion_number > total_amount:
        return 0.0  # this should not be possible
    else:
        return portion_number / total_amount


def function_1(remaining_dirt_number, total_spawned_dirt_number):
    # the less dirt remains from the spawned dirt, the better it is
    # if no dirt remains, output will be 1
    # if all dirt remains, output will be 0
    # total_spawned_dirt_number is the accumulated dirt number spawned since the beginning
    if remaining_dirt_number is None or total_spawned_dirt_number is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif total_spawned_dirt_number == 0:
        return 1.0  # robot cannot clean anything when there is nothing
    elif remaining_dirt_number < 0 or total_spawned_dirt_number < 0 or remaining_dirt_number > total_spawned_dirt_number:
        return 0.0  # this should not be possible
    else:
        return (1 / total_spawned_dirt_number) * (total_spawned_dirt_number - remaining_dirt_number)


def function_2(remaining_dirt_number, elapsed_time, avg_dirt_spawn_freq):
    # the less dirt remains from the probably spawned dirt, the better it is
    # if no dirt remains, output will be 1
    # if all dirt remains, output will be 0
    # avg_dirt_spawn_freq should be in dirt/sec and elapsed_time in sec
    if remaining_dirt_number is None or elapsed_time is None or avg_dirt_spawn_freq is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif avg_dirt_spawn_freq == 0 or elapsed_time == 0:
        return 1.0  # robot cannot clean anything when there is nothing
    elif remaining_dirt_number < 0 or elapsed_time < 0 or avg_dirt_spawn_freq < 0 or remaining_dirt_number > (elapsed_time * avg_dirt_spawn_freq):
        return 0.0  # this should not be possible
    else:
        max_dirt_number = avg_dirt_spawn_freq * elapsed_time
        return (1 / max_dirt_number) * (max_dirt_number - remaining_dirt_number)


def function_3(traveled_distance, elapsed_time, max_possible_velocity, robot_count):
    # the less the robots move, the less they consume energy and the better it is
    # if no robot moves at all, output will be 1
    # if all robots move as much as possible, output will be 0
    if traveled_distance is None or elapsed_time is None or max_possible_velocity is None or robot_count is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif max_possible_velocity == 0 or elapsed_time == 0 or robot_count == 0:
        return 1.0  # robot cannot move if they have no pos. velocity or if there is no robot at all or no time has passed
    elif max_possible_velocity < 0 or elapsed_time < 0 or robot_count < 0:
        return 0.0   # this should not be possible
    else:
        max_possible_distance = max_possible_velocity * elapsed_time * robot_count
        # the travelled_distance can sometimes be not perfectly accurate and more than max_possible_distance, but the score should still be between 0 and 1 (--> min, max)
        return min(1.0, max(0.0, (1 / max_possible_distance) * (max_possible_distance - traveled_distance)))


def function_4(input_to_check, threshold):
    if input_to_check is None or threshold is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif input_to_check > threshold:
        return 0.0
    else:
        return 1.0


def function_5(input_to_check, offset, aim, factor, start_requirement):
    if input_to_check is None or offset is None or aim is None or factor is None or start_requirement is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif aim >= start_requirement and (input_to_check - offset) < (aim * factor):
        return 0.0
    else:
        return 1.0


def function_6(portion_number, total_amount):
    # inverted version of function_0
    # the lower the portion is in comparison to the total amount, the better it is
    # if it is everything, output will be 0
    # if it is nothing, output will be 1
    if portion_number is None or total_amount is None:
        return 1.0  # if input is missing (can be the case in the beginning)
    elif total_amount == 0:
        return 1.0  # it is not possible to have a portion of nothing
    elif portion_number < 0 or total_amount < 0 or portion_number > total_amount:
        return 0.0  # this should not be possible
    else:
        return (total_amount - portion_number) / total_amount
