import math
# ------------------------
# Robot Configurations
# ------------------------
ROBOT_LINEAR_SPEED = 0.25  # meters/second
ROBOT_ANGULAR_SPEED = (math.pi/3)# π/3  (≈1.05 rad/s)
ROBOT_DISTANCE_UNIT = 1.5  # meters
# ------------------------
# Robot Actions
# ------------------------
ROBOT_ACTIONS = {
    "LEFT_TO_RIGHT": "LEFT_TO_RIGHT",
    "LEFT_TO_MIDDLE_TO_LEFT": "LEFT_TO_MIDDLE_TO_LEFT",
    "RIGHT_TO_LEFT": "RIGHT_TO_LEFT",
    "STOP": "STOP"
}

# ------------------------
# Topic Names
# ------------------------
TOPIC_START_ROBOT_MOVEMENT = "start_robot_movement"
TOPIC_LIVE_ACTION = "live_action"
