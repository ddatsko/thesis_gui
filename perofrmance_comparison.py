import typing

import own_planner
import popcorn_planner
import gstp_planner
from utils import get_path_properties
from math import pi
from data_storage import save_paths
import os
import pandas as pd
import time

EXPERIMENTS_LOG_FILE = 'experiments.csv'
EXPERIMENTS_LOG_FILE_HEADER = ['experiment_name', 'algorithm', 'energy', 'path_time', 'path_length', 'n_turns', 'computation_time', 'max_energy', 'min_energy', 'n_paths']


def _create_experiments_log_file():
    if os.path.exists(EXPERIMENTS_LOG_FILE):
        return
    open(EXPERIMENTS_LOG_FILE, 'w').write(','.join(EXPERIMENTS_LOG_FILE_HEADER) + '\n')


def get_total_paths_metrics(paths):
    energy = time = length = num_of_turns = 0.0
    min_energy = float('inf')
    max_energy = -min_energy

    for path in paths:
        path_energy, path_time, path_length, turns = get_path_properties(path)
        energy += path_energy
        time += path_time
        length += path_length
        num_of_turns += turns
        min_energy = min(min_energy, path_energy)
        max_energy = max(max_energy, path_energy)

    return {'energy': energy, 'min_energy': min_energy, 'max_energy': max_energy, 'n_paths': len(paths), 'path_time': time, 'path_length': length, 'n_turns': num_of_turns}


def _test_own_algorithm_multiple_angles(json_data):
    best_paths = []
    best_res = {}
    angle = 0
    while angle < pi:
        json_data['init-rotation'] = angle
        paths, angle_res = run_one_algorithm(json_data, own_planner.plan_paths_own)

        print(f'For angle {angle}, result is {angle_res}')

        if not best_res or angle_res['energy'] < best_res['energy']:
            best_paths = paths
            best_res = angle_res
        angle += 0.3
    return best_paths, best_res


def run_one_algorithm(json_data, algorithm: typing.Callable) -> (list, dict):
    start = time.time_ns()
    paths = algorithm(json_data)
    end = time.time_ns()
    total_path_metrics = get_total_paths_metrics(paths)
    total_path_metrics['computation_time'] = end - start

    return paths, total_path_metrics



def compare_algorithm(json_data, write_data=False, experiment_dir=''):
    json_data['n-uavs'] = 1
    own_paths_1, own_res_1 = _test_own_algorithm_multiple_angles(json_data)
    json_data['n-uavs'] = 3
    own_paths_3, own_res_3 = _test_own_algorithm_multiple_angles(json_data)

    gtsp_paths, gtsp_res = run_one_algorithm(json_data, gstp_planner.plan_path_gtsp)
    popcorn_paths, popcorn_res = run_one_algorithm(json_data, popcorn_planner.plan_paths_wadl)


    if write_data:
        save_paths(own_paths_1, experiment_dir, 'own_1_uav')
        save_paths(own_paths_3, experiment_dir, 'own_3_uavs')
        save_paths(gtsp_paths, experiment_dir, 'gtsp')
        save_paths(popcorn_paths, experiment_dir, 'popcorn')

    print(f'GTSP: {gtsp_res}')
    print(f'POPCORN: {popcorn_res}')
    print(f'OWN 1 UAV: {own_res_1}')
    print(f'OWN 3 UAVs: {own_res_3}')
    print()

    # If experiment has a name - save results to the file
    if experiment_dir:
        _create_experiments_log_file()
        df = pd.read_csv(EXPERIMENTS_LOG_FILE)
        # Remove rows of the same experiment
        df = df[df['experiment_name'] != experiment_dir]

        # Append four experiments
        df = df.append({'experiment_name': experiment_dir, 'algorithm': 'gtsp', **gtsp_res}, ignore_index=True)
        df = df.append({'experiment_name': experiment_dir, 'algorithm': 'popcorn', **popcorn_res}, ignore_index=True)
        df = df.append({'experiment_name': experiment_dir, 'algorithm': 'own_1', **own_res_1}, ignore_index=True)
        df = df.append({'experiment_name': experiment_dir, 'algorithm': 'own_3', **own_res_3}, ignore_index=True)

        df.to_csv(EXPERIMENTS_LOG_FILE, index=False)

    return own_paths_1

