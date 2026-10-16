
# PLACEHOLDER VALUES!
avg_seconds_p_tile  = 15
avg_score_p_tile    = 14
avg_tiles_after_r3  = 14
avg_tiles_before_r3 = 12

# Room 3 state chances
probabilitie_of_room3_having_a_obstacle = 0.5
probabilitie_of_room3_per_shape = {"3x3": 0.34, "3x4": 0.33, "4x3": 0.33}


avg_time_per_action = {"move":6.0, "turn": 9.0, "measure": 0.0}
score_multiplier_per_victim = 1.3

round_duration = 600

avg_score_before_r3 = avg_tiles_before_r3 * avg_score_p_tile
avg_time_before_r3  = avg_tiles_before_r3 * avg_seconds_p_tile
avg_score_after_r3  = avg_tiles_after_r3 * avg_score_p_tile
avg_time_after_r3   = avg_tiles_after_r3 * avg_seconds_p_tile

