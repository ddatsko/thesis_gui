from flask import Flask, render_template, request
import json
import rospy
from thesis_trajectory_generator.srv import GeneratePaths, GeneratePathsRequest, GeneratePathsResponse
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon


def generate_drones_paths(generate_req) -> GeneratePathsResponse or None:
    rospy.wait_for_service("/generate_paths")
    try:
        proxy = rospy.ServiceProxy("/generate_paths", GeneratePaths)
        res = proxy(generate_req)
        print(res)
        return res
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


app = Flask(__name__)

@app.route('/')
def hello_world():
    return render_template('google.html')


@app.route('/generate_trajectories', methods=['POST'])
def generate_trajectories():
    json_data = json.loads(request.data.decode('utf-8'))
    generate_req = GeneratePathsRequest()
    generate_req.fly_zone.points = [Point32(float(x), float(y), 0.0) for x, y in json_data["fly-zone"]]
    generate_req.no_fly_zones = [Polygon([Point32(float(x), float(y), 0.0) for x, y in pol]) for pol in json_data["no-fly-zones"]]
    generate_req.start_lat = float(json_data["start-point"][0])
    generate_req.start_lon = float(json_data["start-point"][1])
    generate_req.sweeping_step = float(json_data["sweeping-step"])
    # TODO: check this. Maybe, give the choice to user
    generate_req.no_improvement_cycles_before_stop = 100
    generate_req.override_drone_parameters = bool(json_data["override-drone-spec"])
    if generate_req.override_drone_parameters:
        print("HERE")
        generate_req.drone_area = float(json_data["uav-area"] or 0)
        generate_req.propeller_radius = float(json_data["propeller-radius"] or 0)
        generate_req.number_of_propellers = int(json_data["n-propellers"] or 0)
        generate_req.drone_mass = float(json_data["uav-mass"] or 0)
    generate_req.override_battery_model = bool(json_data["override-battery-model"])
    if generate_req.override_battery_model:
        generate_req.battery_cell_capacity = float(json_data["cell-capacity"] or 0)
        generate_req.battery_number_of_cells = int(json_data["cells-in-series"] or 0)

    generate_drones_paths(generate_req)

    return "", 200



if __name__ == '__main__':
    app.run()
