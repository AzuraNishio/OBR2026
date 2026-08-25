class line_follower_constants:

    # Maximum linear speed for the sensor in mm/s
    max_sensor_linear_speed = 160

    # Reduced linear speed for the sensor for more precise control in mm/s
    sensor_reduced_linear_speed = 80

    # Duration for the sensor linear speed ramp-up and ramp-down
    sensor_linear_speed_ramp_duration = 0.5

    # Constants for the gap check maneuver in mm
    gap_check_distance = 220

    # Minimum value for the silver detection in %
    silver_min_threshold = 95

    # Side to go around the obstacle, -1 for left, 1 for right
    obstacle_side = -1

    # PD controller coefficients
    KP = 4.0
    KD = 1.0
    KD2 = 1.0

    # Line follower bias to compensate for sensor misalignment or other factors
    bias = -4
