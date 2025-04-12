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
#define CLIMBER_CAN_ID              17       
#define CLIMBER_BEAM_BREAK_ID       1
#define CLIMBER_SPARK_CAN_ID        18

//********** ClAW CONSTANTS **********//

#define CLAW_PHOTO_EYE_FIRST        7
#define CLAW_CAN_ID                 24
#define ALGAE_PHOTO_EYE             3




//Pivot
#define ALG_MOTOR_CAN_ID                25
#define PIVOT_FALCON_CAN_ID         51



//********** ELEVATOR CONSTANTS **********//

#define ELEVATOR_CAN_ID             50
#define ELEV_HOME_SENSOR            9
