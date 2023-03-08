import os
import json


FLY_ZONE_FILENAME = 'fly_zone.csv'
NO_FLY_ZONES_FILENAME_TEMPLATE = 'no_fly_zone_{}.csv'
START_POINT_FILENAME = 'start_point.csv'
PATHS_FOLDER_PREFIX = 'path_'
CONFIG_EXCLUDED_FIELDS = ['fly-zone', 'no-fly-zones', 'start-point']
CONFIG_JSON_FILENAME = 'config.json'
EXPERIMENTS_DIR = '../experiments/'


def _save_points_to_file(polygon, filename):
    with open(filename, 'w') as f:
        for point in polygon:
            print(f'{point[0]}, {point[1]}', file=f)


def _read_points_from_file(filename: str):
    with open(filename, 'r') as f:
        return list(map(lambda line: tuple(map(float, line.split(','))), filter(lambda x: x, f.readlines())))


def save_config(directory, json_data):
    directory = os.path.dirname(__file__) + '/' + EXPERIMENTS_DIR + directory
    if not os.path.exists(directory):
        os.mkdir(directory)

    config_filename = directory + '/' + CONFIG_JSON_FILENAME
    config_file_data = {key: value for key, value in json_data.items()}
    with open(config_filename, 'w') as f:
        json.dump(config_file_data, f)


def read_config(directory) -> dict:
    filename = os.path.dirname(__file__) + '/' + EXPERIMENTS_DIR + '/' + directory.rstrip('/') + '/' + CONFIG_JSON_FILENAME
    if not os.path.exists(filename):
        return dict()
    with open(filename, 'r') as f:
        data = json.load(f)
        return data
