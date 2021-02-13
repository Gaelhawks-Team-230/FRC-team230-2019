#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#define MIN_PLATFORM_ACCELERATION_CENTER           (-3.0 * LOOPTIME)
#define MAX_PLATFORM_ACCELERATION_CENTER           (3.0 * LOOPTIME)
#define MIN_PLATFORM_ACCELERATION_SIDE      (-5.0 * LOOPTIME)
#define MAX_PLATFORM_ACCELERATION_SIDE      (5.0 * LOOPTIME)
#define AUTO_DRIVE_MIN_ACCELERATION         (-5.0 * LOOPTIME)
#define AUTO_DRIVE_MAX_ACCELERATION         (5.0 * LOOPTIME)

#define TARGET_CENTER_VALUE                    (200)
#ifdef PRACTICE_BOT
#define LINE_CENTER_VALUE                   (1.22) //(1.27)//(1.53)
#else
#define LINE_CENTER_VALUE                   (1.21)//(1.09)
#endif
#define TARGET_ROTATE_MULTIPLIER            (-0.01)
#define TARGET_STRAFE_MULTIPLIER            (-0.03)
#define LINE_ROTATE_MULTIPLIER              (-1.0)
#define LINE_STRAFE_MULTIPLIER              (-1.5)
#define TIME_CHECK_FOR_TARGETTING           (5.0 * N1SEC)//(2.0 * N1SEC)
#define TARGET_CENTER_ERROR                 (20)
#define LINE_CENTER_ERROR                   (0.2)

#define ONE_SEC                             (1.0 * N1SEC)
#define HALF_SEC                            (0.5 * N1SEC)
#define ONE_P_FIVE_SEC                      ((int)(1.5 * N1SEC))
#define TWO_SEC                             ((int)(2.0 * N1SEC))
#define ZERO_POINT_8_SEC                       ((int)(0.8 * N1SEC))
#define THREE_SEC                           (3.0 * N1SEC)

#define BASELINE_DISTANCE_FROM_CENTER       (70.0)
#define BASELINE_DISTANCE_FROM_SIDE         (85.0)
#define DISTANCE_OFF_CENTER                 (35.0)

#define DISTANCE_OFF_LEV_2_FOR_ROCKET       (145.0)
#define DISTANCE_TO_ROCKET_AFTER_TURN       (110.0)
#define DISTANCE_INTO_ROCKET                (8.0)

#define END_HATCH_PANEL_FROM_CENTER_DISTANCE    (115.0)
#define DISTANCE_FOLLOWING_LINE             (25.0 + END_HATCH_PANEL_FROM_CENTER_DISTANCE)
#define FAST_FORWARD_DISTANCE               (5.0 + DISTANCE_FOLLOWING_LINE)
#define RELEASE_TIME_LIMIT                  ((int)(2.0 * N1SEC))
#define BACK_DISTANCE_AFTER_PLACEMENT_FROM_END_CENTER_MODE  (10.0)
#define BACK_DISTANCE_AFTER_PLACEMENT_FROM_END_SIDE_MODE  (-20.0)

#define END_HATCH_PANEL_FROM_SIDE_DISTANCE              (100.0)
#define DISTANCE_AFTER_TURN_SIDE_TO_END                 (15.0)
#define DISTANCE_INTO_END_BAY_FROM_SIDE                 (35.0)
#define DISTANCE_OFF_PLATFORM_NEAR_ROCKET               (100.0)
#define DISTANCE_TO_NEAR_ROCKET                         (90.0)
#define DISTANCE_INTO_NEAR_ROCKET                       (10.0)
#define DISTANCE_AWAY_FROM_NEAR_ROCKET                  (16.0)
//??? redefined below??? #define DISTANCE_TO_FEEDER_STATION                      (80.0)
#define DISTANCE_AWAY_FROM_END_BAY                      (16.0)
#define DISTANCE_FROM_END_HATCH_PANEL_TO_SIDE_WALL         (150.0)
#define BACK_DISTANCE_AFTER_NEAR_BAY            (30.0)
#define BACK_DISTANCE_AFTER_MIDDLE_BAY          (20.0)
#define BACK_DISTANCE_AFTER_FAR_BAY             (10.0)
#define DISTANCE_TO_COMMON_END_POINT_NEAR       (110.0)
#define DISTANCE_TO_COMMON_END_POINT_MIDDLE       (60.0)
#define DISTANCE_TO_COMMON_END_POINT_FAR       (70.0)

#define DISTANCE_TO_FEEDER_STATION              (100.0)
#define DISTANCE_AWAY_FROM_FEEDER_STATION       (100.0)

#define DISTANCE_FOR_SIDE_PANEL_FROM_SIDE       (100.0)
#define DISTANCE_AFTER_TURN_SIDE_AUTO       (15.0)//(20.0)//(50.0)
#define SIDE_HATCH_PANEL_NEAR_DISTANCE      (70.0)//(48.0)
#define SIDE_HATCH_PANEL_MIDDLE_DISTANCE    (275.0)
#define SIDE_HATCH_PANEL_FAR_DISTANCE       (300.0)
#define SIDE_DISTANCE_INTO_SHIP             (24.0)

#define BACK_DISTANCE_FROM_FEEDER           (-10.0)
#define BACK_DISTANCE_FOR_END_PANEL         (-50.0)
#define FORWARD_DISTANCE_TO_END_PANEL       (75.0)
#define DISTANCE_BACK_LEV2_END_PANEL        (-10.0)

#define LEV2_HATCH_PANEL_FAR_DISTANCE       (150.0)
#define LEV2_HATCH_PANEL_MIDDLE_DISTANCE    (125.0)
#define LEV2_HATCH_PANEL_NEAR_DISTANCE      (100.0)
#define BACK_DISTANCE_FOR_SIDE_PANEL       (-50.0)
#define DISTANCE_BACK_LEV2_SIDE_PANEL        (-10.0)

#endif /*SAMPLE_H_*/
