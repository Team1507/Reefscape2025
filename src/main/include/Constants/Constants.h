#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "frc/apriltag/AprilTag.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Quaternion.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"

namespace consts::yearspecific {
inline const frc::AprilTagFieldLayout TAG_LAYOUT =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2025ReefscapeWelded);

inline constexpr units::inch_t CLAW_OFFSET_L = 4.75_in;
inline constexpr units::inch_t CLAW_OFFSET_R = 6_in;
inline constexpr frc::Transform2d CLAW_TRANS_L{0_m, CLAW_OFFSET_L,
                                               frc::Rotation2d{}};
inline constexpr frc::Transform2d CLAW_TRANS_R{0_m, CLAW_OFFSET_R,
                                               frc::Rotation2d{}};
}  // namespace consts::yearspecific
                                               


//**********CLIMB CONSTANTS **********//
#define CLIMBER_CAN_ID         17       
#define CLIMBER_BEAM_BREAK_ID  1

//********** ClAW CONSTANTS **********//

#define CLAW_PHOTO_EYE_FIRST   7
#define CLAW_CAN_ID            24
#define ALGAE_PHOTO_EYE        3

//Pivot
#define PIVOT_CAN_ID           25

#define PIVOT_HOME_POSITION    0
#define PIVOT_TOLERANCE        0.05
#define PIVOT_MANUAL_POWER     0.3

//********** ELEVATOR CONSTANTS **********//

#define ELEVATOR_CAN_ID        50
#define ELEV_HOME_SENSOR       9

#define ELEV_PULLY_DIAM 1.25_in

#define ELEV_POSITION_1 -0.05_tr
#define ELEV_POSITION_2 -1.5_tr
#define ELEV_POSITION_3 -3_tr
#define ELEV_POSITION_4 -5.6_tr
#define ELEV_POSITION_5 -5_tr
#define ELEV_POSITION_6 -6_tr

#define ELEV_TOLERANCE 0.05_tr

#define ELEV_MOTOR_PORT 1
#define ELEV_ENCODER_A_CHANNEL 0
#define ELEV_ENCODER_B_CHANNEL 1

#define ELEV_KP 5.0
#define ELEV_GEARING 10.0
#define ELEV_DRUM_RADIUS 0.0508
#define ELEV_CARRING_MASS 4

#define ELEV_MIN_ELEVATOR_HEIGHT 0.0508
#define ELEV_MAX_ELEVATOR_HEIGHT 1.27

#define ELEV_MAX_VELOCITY 1.75
#define ELEV_MAX_ACCEL 0.75

#define ELEV_P 1.3
#define ELEV_I 0.0
#define ELEV_D 0.7
#define ELEV_KDT 0.2

#define ELEV_HOMING_HIGH_SPEED 0.01
#define ELEV_HOMING_CREEP_SPEED 0.005

#define ELEV_MANUAL_SLOW_POWER 0.4
#define ELEV_MANUAL_SLOW_DOWN_POWER -0.1
