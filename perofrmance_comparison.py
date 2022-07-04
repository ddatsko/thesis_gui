import own_planner
import popcorn_planner
import gstp_planner
from utils import get_path_properties
from math import pi


def get_total_paths_metrics(paths):
    energy = time = length = 0

    for path in paths:
        path_energy, path_time, path_length = get_path_properties(path)
        energy += path_energy
        time += path_time
        length += path_length

    return energy, time, length


def compare_algorithm(json_data):
    own_res = (float('inf'), float('inf'), float('inf'))
    angle = 0
    while angle < pi:
        json_data['init-rotation'] = angle
        angle_res = get_total_paths_metrics(own_planner.plan_paths_own(json_data))
        print(f'For angle {angle}, result is {angle_res}')
        if angle_res[0] < own_res[0]:
            own_res = angle_res
        angle += 0.3

    gstsp_res = get_total_paths_metrics(gstp_planner.plan_path_gtsp(json_data))
    popcorn_res = get_total_paths_metrics(popcorn_planner.plan_paths_wadl(json_data))
    print(f'GSTSP: {gstsp_res}')
    print(f'POPCORN: {popcorn_res}')
    print(f'OWN: {own_res}')

