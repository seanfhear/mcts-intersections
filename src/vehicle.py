import yaml
import traci
import numpy as np
from passing_order import PassingOrder
import monte_carlo

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)


class Vehicle:
    def __init__(self, veh_id, route, state, veh_data=None):
        if veh_data is None:
            veh_data = {}
        self.veh_id = veh_id
        self.route = route
        self.state = state
        self.veh_data = veh_data

        traci.vehicle.add(veh_id, route)
        self.set_vehicle_state(state)

    def set_vehicle_state(self, state):
        self.state = state
        if state == cfg["veh_state_default"]:
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_grey"])
            traci.vehicle.setSpeedMode(self.veh_id, cfg["veh_safe_mode"])
            traci.vehicle.setSpeed(self.veh_id, cfg["veh_speed_default"])

        elif state == cfg["veh_state_control"]:
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_green"])

        elif state == cfg["veh_state_intersection"]:
            traci.vehicle.setColor(self.veh_id, cfg["veh_col_white"])
            traci.vehicle.setSpeed(self.veh_id, cfg["veh_speed_intersection"])

    def is_outbound(self):
        return traci.vehicle.getRoadID(self.veh_id) == traci.vehicle.getRoute(self.veh_id)[-1]

    def get_dist_to_intersection(self, veh_id=None):
        if veh_id is None:
            veh_id = self.veh_id

        loc = np.array(traci.vehicle.getPosition(veh_id))
        intersection_loc = np.array(cfg["intersection_coords"])

        return np.linalg.norm(loc - intersection_loc)

    def gather_veh_data(self, vehicles):
        self.veh_data = {}
        for veh in vehicles:
            self.veh_data[veh.veh_id] = {
                "route": veh.route[6:],
                "edge": traci.vehicle.getLaneID(veh.veh_id),
                "distance": float(self.get_dist_to_intersection(veh.veh_id)),
                "adjustment": 0
            }

        return self.veh_data

    def get_passing_order(self):
        passing_order = None
        if cfg["passing_order_mode"] == cfg["passing_order_fcfs"]:
            passing_order = PassingOrder(sorted(self.veh_data.items(), key=lambda item: item[1]["distance"]))
        elif cfg["passing_order_mode"] == cfg["passing_order_mcts"]:
            passing_order = PassingOrder(monte_carlo.get_passing_order(self.veh_data))

        passing_order.calculate_adjustments()

        return passing_order
