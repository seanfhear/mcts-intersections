import traci
import yaml

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)


def safe_route(route_a, route_b):
    combined_a = [int(route_a), int(route_b)]
    combined_b = [int(route_b), int(route_a)]

    return combined_a not in cfg["colliding_routes"] and combined_b not in cfg["colliding_routes"]


def safe_distance(dist_a, dist_b):
    return abs(dist_a - dist_b) > cfg["collision_distance_thresh"]


class PassingOrder:
    def __init__(self, passing_order):
        self.passing_order = passing_order
        self.adjusted_order = {}
        self.total_adjustment = 0

        for veh in passing_order:
            self.adjusted_order[veh[0]] = 0

    def calculate_adjustments(self):
        for i, veh in enumerate(self.passing_order):
            adjustment_required = self.is_adjustment_required(i)
            while adjustment_required:
                for v in self.passing_order[i:]:
                    if traci.vehicle.getRoadID(veh[0]) == traci.vehicle.getRoadID(v[0]):
                        self.adjusted_order[v[0]] += 1
                        self.total_adjustment += 1

                adjustment_required = self.is_adjustment_required(i)

    def is_adjustment_required(self, i):
        """
        Whether a given vehicle is on a colliding trajectory with any vehicle ahead of it in the passing order
        """
        offset = cfg["collision_distance_thresh"]
        for j in range(i - 1, -1, -1):
            route_a = self.passing_order[i][1]["route"]
            route_b = self.passing_order[j][1]["route"]

            dist_a = self.passing_order[i][1]["distance"] + (offset * self.adjusted_order[self.passing_order[i][0]])
            dist_b = self.passing_order[j][1]["distance"] + (offset * self.adjusted_order[self.passing_order[j][0]])

            adj_required = not safe_route(route_a, route_b) and not safe_distance(dist_a, dist_b)
            if adj_required:
                return adj_required

    def to_string(self):
        return ",".join([item[0] for item in self.passing_order])
