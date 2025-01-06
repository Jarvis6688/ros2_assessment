from enum import Enum

# Define robot states
class State(Enum):
    FORWARD = 0  # Moving forward
    TURNING = 1  # Turning
    COLLECTING = 2  # Collecting items
    DROPPING = 3  # Dropping items
    DELIVERING_NAV2 = 4  # Delivering items using Nav2
    INITIAL_NAVIGATION_ROBOT = 5  # Initial navigation to work area
