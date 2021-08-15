import os
import sys
import yaml
import traci
from traci.exceptions import TraCIException
from random import randint
from vehicle import Vehicle
from passing_order import PassingOrder
from reporting import show_report

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)

sumoCmd = [
    cfg["sumo_binary"],
    '-c', cfg["sumo_cfg"],
    '--tripinfo-output', cfg["trip_info_out"],
    '--collision-output', cfg["collisions_out"],
    '--statistic-output', cfg["statistics_out"]
]


def add_vehicles(num_vehicles):
    vehicles = []
    for i in range(num_vehicles):
        veh_id = "veh_{}".format(i)
        route = "route_{}".format(randint(0, cfg["num_routes"] - 1))

        veh = Vehicle(
            veh_id=veh_id,
            route=route,
            state=cfg["veh_state_default"]
        )
        vehicles.append(veh)

    return vehicles


def get_zone_radii():
    z1 = cfg["zone_control_size"]
    z2 = cfg["zone_intersection_size"]

    z1_rad = z2 + z1

    return z1_rad, z2


def execute_adjustments(passing_order):
    try:
        for veh in passing_order.adjusted_order:
            if passing_order.adjusted_order[veh] == 0:
                traci.vehicle.slowDown(veh, cfg["veh_speed_default"], 0)
            else:
                traci.vehicle.slowDown(veh, cfg["veh_speed_adjust"], 0)
                traci.vehicle.setColor(veh, cfg["veh_col_orange"])
                passing_order.adjusted_order[veh] -= 0.25
    except TraCIException:
        pass


def main():
    traci.start(sumoCmd)
    vehicles = add_vehicles(cfg["num_vehicles"])
    passing_order = None
    highest_total = 0

    step = 0
    while step < cfg["max_simulation_steps"]:
        traci.simulationStep()
        step += 1

        z1_radius, z2_radius = get_zone_radii()

        for veh in vehicles:
            try:
                if veh.is_outbound():
                    veh.set_vehicle_state(cfg["veh_state_default"])
                else:
                    if veh.get_dist_to_intersection() < z2_radius:
                        veh.set_vehicle_state(cfg["veh_state_intersection"])
                    elif veh.get_dist_to_intersection() < z1_radius:
                        veh.set_vehicle_state(cfg["veh_state_control"])

            except TraCIException:  # vehicle has left the sim
                vehicles.remove(veh)

        if step == 20:
            if cfg["passing_order_mode"] == cfg["passing_order_fcfs"]:
                vehicles[0].gather_veh_data(vehicles)
                passing_order = vehicles[0].get_passing_order()
            elif cfg["passing_order_mode"] == cfg["passing_order_mcts"]:
                votes = {}
                for veh in vehicles:
                    veh.gather_veh_data(vehicles)
                    passing_order = vehicles[0].get_passing_order()
                    vote = passing_order.to_string()

                    if vote in votes:
                        votes[vote]["score"] += 1
                    else:
                        votes[vote] = {
                            "score": 1,
                            "order": passing_order
                        }

                highest_total = max(votes.items(), key=lambda item: item[1]["score"])[1]["score"]
                result = None
                for entry in votes.items():
                    if entry[1]["score"] == highest_total:
                        if result is None:
                            result = {
                                "order": entry[1]["order"],
                                "adjust": entry[1]["order"].total_adjustment
                            }
                        else:
                            if entry[1]["order"].total_adjustment < result["adjust"]:
                                result = {
                                    "order": entry[1]["order"],
                                    "adjust": entry[1]["order"].total_adjustment
                                }

                passing_order = result["order"]

        if step > 25:
            execute_adjustments(passing_order)

    traci.close()

    show_report(passing_order, highest_total)


if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("You must declare environment variable SUMO_HOME")

    main()
