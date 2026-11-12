from V3.obrExperimentalData import *
from V3.NekoMath import *
from V3.r3_info import r3_info
from V3.r3_model_robot import r3_model_robot

def info_robot_score_function(candidate_data, info, robot):
    return 0

def room_robot_score_functions(robot: r3_model_robot, candidate_data: dict):
    delivered_victims = candidate_data["delivered_victims"]
    onboard_victims = candidate_data["onboard_victims"]

    time_from_round_beggining = avg_time_before_r3 + robot.time
    time_to_round_end = round_duration - time_from_round_beggining

    """
    In order to know what score we'll be gaining by delivering the victims, we 
    need to know the total score to multiply. Thats why we calculate the expected score
    after r3! Furthermore, if by delivering the victims the robot exits too late, so
    that we get few points after r3, them it gets less rewarded!
    """

    potencial_score_after_r3 = avg_score_after_r3 * saturate(time_to_round_end, avg_time_after_r3 * 0.2)
    # potencial_score_after_r3 += 60 * nk.sigmoid(time_to_round_end - avg_time_after_r3, 3)
    expected_score = avg_score_before_r3 + potencial_score_after_r3
    expected_score_multiplier = pow(score_multiplier_per_victim, delivered_victims)

    score_before_onboard = expected_score * expected_score_multiplier
    non_exit_score = score_before_onboard * pow(
        score_multiplier_per_victim - 0.05,
        onboard_victims,
    )
    exit_score = score_before_onboard + 10 * sigmoid(delivered_victims - 3)
    return non_exit_score, exit_score