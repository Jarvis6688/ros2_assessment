import math

# Robot motion parameters
LINEAR_VELOCITY = 0.3  # Linear speed (m/s)
ANGULAR_VELOCITY = 0.3  # Angular speed (rad/s)

# Turning direction
TURN_LEFT = 1  # Left turn
TURN_RIGHT = -1  # Right turn

# Lidar parameters
SCAN_THRESHOLD = 0.4  # Obstacle distance (m)
SCAN_WARN_THRESHOLD = 0.46  # Warning distance (m)
SCAN_FRONT = 0  # Front area
SCAN_LEFT = 1  # Left area
SCAN_BACK = 2  # Back area
SCAN_RIGHT = 3  # Right area

# Zones with coordinates, yaw, and color
ZONES = {
    'BOTTOM_RIGHT': {
        'color': 'RED',
        'x': -3.42,
        'y': -2.46,
        'target_yaw': math.pi / 20
    },
    'TOP_RIGHT': {
        'color': 'GREEN',
        'x': 2.37,
        'y': -2.5,
        'target_yaw': math.pi * 5 / 6
    },
    'TOP_LEFT': {
        'color': 'BLUE',
        'x': 2.37,
        'y': 2.46,
        'target_yaw': -3 * math.pi / 4
    },
    'BOTTOM_LEFT': {
        'color': 'BLACK',
        'x': -3.42,
        'y': 2.46,
        'target_yaw': -math.pi / 4
    }
}
