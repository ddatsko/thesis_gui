import os
import pathlib
import shutil

FLY_ZONE_FILENAME = 'fly_zone.csv'
NO_FLY_ZONES_FILENAME_TEMPLATE = 'no_fly_zone_{}.csv'
START_POINT_FILENAME = 'start_point.csv'
PATHS_FOLDER_PREFIX = 'path_'


def save_points_to_file(polygon, filename):
    with open(filename, 'w') as f:
        for point in polygon:
            print(f'{point[0]}, {point[1]}', file=f)


def read_points_from_file(filename: str):
    with open(filename, 'r') as f:
        return list(map(lambda line: tuple(map(float, line.split(','))), filter(lambda x: x, f.readlines())))


def save_polygon(fly_zone, no_fly_zones, start_point, directory):
    # Make an empty directory <project_name>/polygon or remove all the old files from there
    if not os.path.exists(directory):
        os.mkdir(directory)
    polygon_dir = directory + '/polygon'
    if os.path.exists(polygon_dir):
        for file in os.listdir(polygon_dir):
            os.remove(polygon_dir + '/' + file)
    else:
        os.mkdir(polygon_dir)

    save_points_to_file(fly_zone, polygon_dir + '/' + FLY_ZONE_FILENAME)
    save_points_to_file([start_point], polygon_dir + '/' + START_POINT_FILENAME)
    for i in range(len(no_fly_zones)):
        save_points_to_file(no_fly_zones[i], polygon_dir + '/' + NO_FLY_ZONES_FILENAME_TEMPLATE.format(i))


def save_paths(paths, directory, create_new_dir=True):
    if not os.path.exists(directory):
        os.mkdir(directory)

    path_dir = f"{directory}/{PATHS_FOLDER_PREFIX}0"
    if not create_new_dir:
        for filename in os.listdir(directory):
            if filename.startswith(PATHS_FOLDER_PREFIX):
                shutil.rmtree(directory + f'/{filename}')
    else:
        i = 0
        while os.path.exists(directory + f'/{PATHS_FOLDER_PREFIX}{i}'):
            i += 1
        path_dir = f"{directory}/{PATHS_FOLDER_PREFIX}{i}"
    os.mkdir(path_dir)
    for i in range(len(paths)):
        save_points_to_file(paths[i], f"{path_dir}/path_{i}.csv")
        i += 1


def load_paths_from_dir(directory: str):
    directory = directory.rstrip('/') + '/'
    if not os.path.exists(directory):
        return "Invalid directory path", 500
    files = os.listdir(directory)
    if FLY_ZONE_FILENAME not in files:
        return "No fly zone file found in the directory", 500
    if START_POINT_FILENAME not in files:
        return "No start point found in the directory", 500
    fly_zone = read_points_from_file(directory + FLY_ZONE_FILENAME)
    start_point = read_points_from_file(directory + START_POINT_FILENAME)[0]

    no_fly_zones = []
    i = 0
    while os.path.exists(filename := (directory + NO_FLY_ZONES_FILENAME_TEMPLATE.format(i))):
        no_fly_zones.append(read_points_from_file(filename))

    return {'fly-zone': fly_zone, 'no-fly-zones': no_fly_zones, 'start-point': start_point}





