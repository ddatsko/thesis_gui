#!/usr/bin/python
from flask import Flask, render_template, request
import json
from scripts.trajectory_planners.own_planner import plan_paths_own
from scripts.utils import *
from threading import Thread

app = Flask(__name__)

last_generated_paths_global = None
last_generated_paths_method = ""

@app.route('/')
def hello_world():
    return render_template('map.html')


@app.route('/generate_trajectories', methods=['POST'])
def generate_trajectories():
    try:
        json_data = json.loads(request.data.decode('utf-8'))

        paths = []
        algorithm = json_data['planning-algorithm']
        if algorithm == 'own':
            paths = plan_paths_own(json_data)

        if not paths:
            return 'Error: No paths were generated', 500

        # If paths were produced by not own algorithm -- create a list of PathSrv messages o
        global last_generated_paths_method, last_generated_paths_global
        last_generated_paths_method = algorithm

        energies = []
        times = []
        lengths = []

        for path in paths:
            energy, time, length = 0, 0, 0 
            energies.append(energy)
            times.append(time)
            lengths.append(length)

        print(f"Total energy: {sum(map(float, energies))}",
              f"Total time: {sum(map(float, times))}",
              f"Total distance: {sum(map(float, lengths))}", sep='\n')

        response = json.dumps({
            'success': True,
            'path': paths,
            'energies': energies,
            'times': times,
            'lengths': lengths
        })
        return response
    except Exception as e:
        print("ERROR: ", e, sep='\n')
        return f"Error: {e}", 500


def load_paths_thread(path, service, res, ind):
    success, msg = send_path_to_service(path, service)
    res[ind] = [len(res), "SUCCESS" if success else "ERROR", "Message: " + msg]


@app.route('/load_paths', methods=['POST'])
def load_paths():

    json_data = json.loads(request.data.decode('utf-8'))
    paths_to_load = json_data["uav_topic"]
    services_used = set()
    res = [[] for _ in range(len(paths_to_load))]

    global last_generated_paths_method, last_generated_paths_global
    if last_generated_paths_method == 'own':
        last_generated_paths = get_last_own_paths()
    else:
        last_generated_paths = last_generated_paths_global

    i = -1
    threads = []
    for path_ind, service in paths_to_load:
        i += 1
        if not service or service in services_used:
            res[i] = [len(res), "ERROR", "Empty service name"]
            continue
        services_used.add(service)

        thread = Thread(target=load_paths_thread, args=(last_generated_paths[path_ind], service, res, i))
        thread.start()
        threads.append(thread)

    for i in range(len(threads)):
        threads[i].join()

    return json.dumps(res), 200


@app.route('/get_services')
def get_services():
    try:
        valid_services = tuple(map(lambda x: x[0],
                                   filter(lambda x: x[1] == "mrs_msgs/PathSrv",
                                          map(lambda x: (x, rosservice.get_service_type(x)) if 'logger' not in x and 'mavros' not in x else (
                                              "", ""), rosservice.get_service_list()))))
    except Exception as e:
        print(e)
        return json.dumps({'valid_services': []}), 200
    return json.dumps({"valid_services": valid_services}), 200


if __name__ == '__main__':
    app.run(host="127.0.0.1", port=5000, debug=True)
