import yaml
from xml.etree import ElementTree

config_file = open("./config.yaml")
cfg = yaml.safe_load(config_file)


def count_collisions():
    collisions_file = cfg["collisions_out"]
    collision_data = ElementTree.parse(collisions_file)
    collisions_root = collision_data.getroot()

    return len(collisions_root.findall('collision'))


def get_duration():
    statistics_file = cfg["statistics_out"]
    statistics_data = ElementTree.parse(statistics_file)
    statistics_root = statistics_data.getroot()

    return statistics_root.find("vehicleTripStatistics").attrib["duration"]


def show_report(passing_order, highest_total):
    print()
    print("Number of Collisions: {}".format(count_collisions()))
    print("Total Required Adjustments: {}".format(passing_order.total_adjustment))
    print("Average Duration: {}".format(get_duration()))
    if highest_total > 0:
        print("Consensus: {}".format(highest_total))
