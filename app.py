from flask import Flask, render_template, request
import json
import rospy
import rosservice
from thesis_path_generator.srv import GeneratePaths, GeneratePathsRequest, GeneratePathsResponse
from mrs_msgs.srv import PathSrv, PathSrvRequest, PathSrvResponse
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

last_generated_paths = []


def generate_drones_paths(generate_req) -> GeneratePathsResponse or None:
    rospy.wait_for_service("/generate_paths")

    proxy = rospy.ServiceProxy("/generate_paths", GeneratePaths)
    res = proxy(generate_req)
    if not res.success:
        return json.dumps({"success": False, "res": res.message})
    else:
        global last_generated_paths
        last_generated_paths = res.paths_gps
        return json.dumps({"success": True,
                            "path": [[(p.position.y, p.position.x) for p in path.points] for path in res.paths_gps],
                            "energy": res.energy_consumptions})


def send_path_to_service(path, service):
    service_list = rosservice.get_service_list()
    if service not in service_list:
        return False, "Service not in service list"
    try:
        follow_path = rospy.ServiceProxy(service, PathSrv)
        path_srv = PathSrvRequest(path)
        res = follow_path(path_srv)
        if not res.success:
            return False, res.message
        else:
            return True, res.message
    except Exception as e:
        return False, "Error while calling service: " + str(e)


app = Flask(__name__)


@app.route('/')
def hello_world():
    return render_template('google.html')


@app.route('/generate_trajectories', methods=['POST'])
def generate_trajectories():
    try:
        json_data = json.loads(request.data.decode('utf-8'))
        generate_req = GeneratePathsRequest()
        generate_req.fly_zone.points = [Point32(float(x), float(y), 0.0) for x, y in json_data["fly-zone"]]
        generate_req.no_fly_zones = [Polygon([Point32(float(x), float(y), 0.0) for x, y in pol]) for pol in json_data["no-fly-zones"]]
        generate_req.start_lat = float(json_data["start-point"][0])
        generate_req.start_lon = float(json_data["start-point"][1])
        generate_req.sweeping_step = float(json_data["sweeping-step"])
        generate_req.number_of_drones = int(json_data["n-uavs"])
        generate_req.rotations_per_cell = int(json_data["rotations-per-cell"])
        generate_req.decomposition_rotation = float(json_data["init-rotation"])
        generate_req.max_polygon_area = float(json_data["max-piece-area"] or 0)
        generate_req.drones_altitude = int(json_data["altitude"])
        generate_req.distance_for_turning = float(json_data["distance-for-rotation"])
        generate_req.max_number_of_extra_points = int(json_data["max-extra-points"])

        # TODO: check this. Maybe, give the choice to user
        generate_req.no_improvement_cycles_before_stop = 100
        generate_req.override_drone_parameters = bool(json_data["override-drone-spec"])
        if generate_req.override_drone_parameters:
            generate_req.drone_area = float(json_data["uav-area"] or 0)
            generate_req.propeller_radius = float(json_data["propeller-radius"] or 0)
            generate_req.number_of_propellers = int(json_data["n-propellers"] or 0)
            generate_req.drone_mass = float(json_data["uav-mass"] or 0)
            generate_req.average_acceleration = float(json_data["acceleration"])
        generate_req.override_battery_model = bool(json_data["override-battery-model"])
        if generate_req.override_battery_model:
            generate_req.battery_cell_capacity = float(json_data["cell-capacity"] or 0)
            generate_req.battery_number_of_cells = int(json_data["cells-in-series"] or 0)

        res = generate_drones_paths(generate_req)
        return res, 200
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
                              map(lambda x: (x, rosservice.get_service_type(x)) if 'logger' not in x else ("", ""), rosservice.get_service_list()))))
    except Exception as e:
        return json.dumps({'valid_services': []}), 200
    return json.dumps({"valid_services": valid_services}), 200




if __name__ == '__main__':
    app.run(host="127.0.0.1", port=5000)
