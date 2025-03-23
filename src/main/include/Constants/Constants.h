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
        frc::AprilTagField::kDefaultField);
inline constexpr units::inch_t CLAW_OFFSET_L = 6.5_in;
inline constexpr units::inch_t CLAW_OFFSET_R = 6_in;
inline constexpr frc::Transform2d CLAW_TRANS_L{0_m, CLAW_OFFSET_L,
                                               frc::Rotation2d{}};
inline constexpr frc::Transform2d CLAW_TRANS_R{0_m, CLAW_OFFSET_R,
                                               frc::Rotation2d{}};
}


//**********CLIMB CONSTANTS **********//
#define CLIMBER_CAN_ID         17       
#define CLIMBER_BEAM_BREAK_ID  1
#define CLIMBER_SPARK_CAN_ID   18

//********** ClAW CONSTANTS **********//

#define CLAW_PHOTO_EYE_FIRST   7
#define CLAW_CAN_ID            24
#define ALGAE_PHOTO_EYE        3

//Pivot
#define PIVOT_CAN_ID           25
#define PIVOT_FALCON_CAN_ID    51

#define PIVOT_HOME_POSITION    0
#define PIVOT_TOLERANCE        0.05
#define PIVOT_MANUAL_POWER     0.3



//********** ELEVATOR CONSTANTS **********//

#define ELEVATOR_CAN_ID        50
#define ELEV_HOME_SENSOR       9

#define ELEV_PULLY_DIAM 1.25_in

#define ELEV_POSITION_HOME -0.03_tr    //home
#define ELEV_POSITION_L1 -0.85_tr    //L1
#define ELEV_POSITION_L2 -1.5_tr     //L2
#define ELEV_POSITION_L3 -3.15_tr       //L3
#define ELEV_POSITION_L4 -5.7_tr     //L4
#define ELEV_POSITION_ALG_L2 -2.23_tr       //ALgea Low
#define ELEV_POSITION_ALG_L3 -4.15_tr       //Algea High

#define ELEV_TOLERANCE 0.05_tr


#define ELEV_HOMING_HIGH_SPEED 0.01
#define ELEV_HOMING_CREEP_SPEED 0.005

#define ELEV_MANUAL_SLOW_POWER 0.4
#define ELEV_MANUAL_SLOW_DOWN_POWER -0.1

#define ELEV_POS_HOME 1
#define ELEV_POS_L1 7
#define ELEV_POS_L2 4
#define ELEV_POS_L3 2
#define ELEV_POS_L4 3
#define ELEV_POS_ALG_LOW 5
#define ELEV_POS_ALG_HIGH 6

// ********* PIVOT CONSTANTS **********//

#define PIVOT_POSITION_HOME 0.29_tr
#define PIVOT_POSITION_OPEN -1.4_tr

#define ALGAE_POS_HOME 1
#define ALGAE_POS_OPEN 2