import math
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import ItemList
from . import constants  # Or import constants from solution


def split_lidar_ranges(msg: LaserScan):
    """
    Split LaserScan data into front, left, back, and right zones.
    :return: Four lists of ranges
    """
    front_ranges = msg.ranges[315:360] + msg.ranges[0:45]  # Front: 315°~45°
    left_ranges = msg.ranges[45:135]  # Left: 45°~135°
    back_ranges = msg.ranges[135:225]  # Back: 135°~225°
    right_ranges = msg.ranges[225:315]  # Right: 225°~315°

    return front_ranges, left_ranges, back_ranges, right_ranges

def get_valid_ranges(ranges_list):
    """
    Remove invalid values (inf or NaN) from a range list.
    :return: Valid ranges
    """
    return [r for r in ranges_list if not math.isinf(r) and not math.isnan(r)]

def get_min_distance(ranges_list):
    """
    Get the minimum valid distance or return infinity if none.
    """
    valid = get_valid_ranges(ranges_list)
    return min(valid) if valid else float('inf')

def process_lidar_data(scan_msg: LaserScan, items: ItemList):
    """
    Process LaserScan data and check for obstacles in each zone.
    Ignore front zone if items are detected.
    :return: Dictionary of distances and triggers
    """
    front_ranges, left_ranges, back_ranges, right_ranges = split_lidar_ranges(scan_msg)

    # Calculate minimum distances
    min_front = get_min_distance(front_ranges)
    min_left  = get_min_distance(left_ranges)
    min_back  = get_min_distance(back_ranges)
    min_right = get_min_distance(right_ranges)

    # Ignore front distance if items exist
    if items and len(items.data) > 0:
        min_front = float('inf')

    # Trigger flags based on thresholds
    front_triggered = (min_front < constants.SCAN_THRESHOLD)
    left_triggered  = (min_left  < constants.SCAN_THRESHOLD)
    back_triggered  = (min_back  < constants.SCAN_THRESHOLD)
    right_triggered = (min_right < constants.SCAN_THRESHOLD)

    return {
        "front_dist": min_front,
        "left_dist":  min_left,
        "back_dist":  min_back,
        "right_dist": min_right,
        "front_triggered": front_triggered,
        "left_triggered":  left_triggered,
        "back_triggered":  back_triggered,
        "right_triggered": right_triggered
    }
