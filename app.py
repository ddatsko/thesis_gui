from flask import Flask, render_template, request
import json
from popcorn_planner import plan_paths_wadl
from own_planner import plan_paths_own
from gstp_planner import plan_path_gtsp
from popcorn_planner import plan_paths_wadl
from utils import *

last_generated_paths = []

app = Flask(__name__)


@app.route('/')
def hello_world():
    return render_template('google.html')


@app.route('/generate_trajectories', methods=['POST'])
def generate_trajectories():
    try:
        json_data = json.loads(request.data.decode('utf-8'))
        paths = []
        algorithm = json_data['planning-algorithm']
        if algorithm == 'own':
            paths = plan_paths_own(json_data)
        elif algorithm == 'gtsp':
            paths = plan_path_gtsp(json_data)
        elif algorithm == 'popcorn':
            paths = plan_paths_wadl(json_data)

        if not paths:
            return 'Error: No paths were generated', 500

        print(paths)

        energies = []
        times = []
        lengths = []

        global last_generated_paths
        last_generated_paths = [[Point32(p[0], p[1], json_data['altitude']) for p in path] for path in paths]

        for path in paths:
            energy, time, length = get_path_properties(path)
            energies.append(energy)
            times.append(time)
            lengths.append(length)

        return json.dumps({
            'success': True,
            'path': paths,
            'energies': energies,
            'times': times,
            'lengths': lengths
        })
    except Exception as e:
        return f"Error: {e}", 500


@app.route('/load_paths', methods=['POST'])
def load_paths():
    json_data = json.loads(request.data.decode('utf-8'))
    paths_to_load = json_data["uav_topic"]
    print("PATHS TO LOAD", paths_to_load)
    services_used = set()
    res = []
    for path_ind, service in paths_to_load:
        if not service or service in services_used:
            res.append([len(res), "ERROR", "Empty service name"])
            continue
        services_used.add(service)
        success, msg = send_path_to_service(last_generated_paths[path_ind], service)
        res.append([len(res), "SUCCESS" if success else "ERROR", "Message: " + msg])
    return json.dumps(res), 200


@app.route('/get_services')
def get_services():
    try:
        valid_services = tuple(map(lambda x: x[0],
                                   filter(lambda x: x[1] == "mrs_msgs/PathSrv",
                                          map(lambda x: (x, rosservice.get_service_type(x)) if 'logger' not in x else (
                                              "", ""), rosservice.get_service_list()))))
    except Exception as e:
        return json.dumps({'valid_services': []}), 200
    return json.dumps({"valid_services": valid_services}), 200


if __name__ == '__main__':
    app.run(host="127.0.0.1", port=5000)
