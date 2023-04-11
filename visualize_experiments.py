import matplotlib.pyplot as plt
import numpy as np
import json
from scripts.utils import gps_coordinates_to_meters
from math import cos, sin

# EXPERIMENTS = ['cape', 'temesvar_nice_8_0', 'rect_8_0', 'temesvar_complex_15', 'temesvar_complex_10', 'temesvar_simple_6']
EXPERIMENTS = ['cape', 'temesvar_nice_8_0', 'rect_8_0', 'temesvar_complex_10', 'temesvar_simple_6']

EXPERIMENT_TO_TITLE = {
    'cape': 'Cape',
    'temesvar_nice_8_0': 'Island',
    'temesvar_complex_10': 'Complex',
    # 'temesvar_complex_15': 'Complex 15',
    'rect_8_0': 'Rectangle',
    'temesvar_simple_6': 'Simple'
}

def rotate_point(x, y, phi):
    return x * cos(phi) - y * sin(phi), x * sin(phi) + y * cos(phi)

def get_experiment_polygon(experiment_name: str):
    config = json.load(open(f'experiments/{experiment_name}/config.json'))
    fly_zone = []
    no_fly_zones = []
    gps_init = config['fly-zone'][0]
    for p in config['fly-zone']:
        fly_zone.append(gps_coordinates_to_meters(p[0], p[1], gps_init[0], gps_init[1]))

    for zone in config['no-fly-zones']:
        no_fly_zones.append([])
        for p in zone:
            no_fly_zones[-1].append(gps_coordinates_to_meters(p[0], p[1], gps_init[0], gps_init[1]))

    start_point_m = gps_coordinates_to_meters(config['start-point'][0], config['start-point'][1], gps_init[0], gps_init[1])

    min_x = min([p[0] for p in fly_zone])
    min_y = min([p[1] for p in fly_zone])

    if experiment_name == 'cape':
        phi = 0.4
        fly_zone = [rotate_point(p[0], p[1], phi) for p in fly_zone]
        start_point_m = rotate_point(start_point_m[0], start_point_m[1], phi)

    return np.array(fly_zone) - np.array([min_x, min_y]), np.array(no_fly_zones), np.array(start_point_m)


def plot_experiment(experiment_name: str, plot: plt.Axes):
    fly_zone, no_fly_zones, start = get_experiment_polygon(experiment_name)
    max_x = int(max([p[0] for p in fly_zone]))
    max_y = int(max([p[1] for p in fly_zone]))

    plot.fill(fly_zone[:, 0], fly_zone[:, 1], facecolor=(0.19, 0.8, 0.19, 1))
    plot.set_xticks([0, max_x])
    plot.set_yticks([0, max_y])
    plot.axis('equal')
    # plot.maxN
    if no_fly_zones.size > 0:
        for i in range(no_fly_zones.shape[0]):
            plot.fill(no_fly_zones[i, :, 0], no_fly_zones[i, :, 1], facecolor=(0.8, 0.19, 0.19, 1))

    plot.scatter([start[0]], [start[1]], s=[100], marker='X', color='black')

    plot.spines['top'].set_visible(False)
    # plot.spines['right'].set_visible(False)
    plot.spines['bottom'].set_visible(False)
    # plot.spines['left'].set_visible(False)

    for item in plot.get_xticklabels() + plot.get_yticklabels():
        item.set_fontsize(8)

    plot.set_title(EXPERIMENT_TO_TITLE[experiment_name])


def visualize_experiments(experiments):
    n = len(experiments)
    fig, subplots = plt.subplots(1, n)
    for i in range(n):
        plot_experiment(experiments[i], subplots[i])

    plt.show()


if __name__ == '__main__':
    visualize_experiments(EXPERIMENTS)