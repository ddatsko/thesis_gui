from thesis_path_generator.srv import GeneratePaths, GeneratePathsRequest, GeneratePathsResponse
from geometry_msgs.msg import Point32, Polygon
import rospy


def _generate_drones_paths_ros(generate_req):
    rospy.wait_for_service("/generate_paths")

    proxy = rospy.ServiceProxy("/generate_paths", GeneratePaths)
    res = proxy(generate_req)
    if not res.success:
        return []
    else:
        return [[(p.position.y, p.position.x) for p in path.points] for path in res.paths_gps]


def plan_paths_own(json_data):
    print("OWN PLANNER")
    generate_req = GeneratePathsRequest()
    generate_req.fly_zone.points = [Point32(float(x), float(y), 0.0) for x, y in json_data["fly-zone"]]
    generate_req.no_fly_zones = [Polygon([Point32(float(x), float(y), 0.0) for x, y in pol]) for pol in
                                 json_data["no-fly-zones"]]
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

    return _generate_drones_paths_ros(generate_req)
