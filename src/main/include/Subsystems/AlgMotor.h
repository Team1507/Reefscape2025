#pragma once

#include <frc2/command/SubsystemBase.h>

#include "constants/Constants.h"
#include "constants/Presets.h"

#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class AlgMotor : public frc2::SubsystemBase {
 public:
  AlgMotor();
  void Periodic() override;

   void SetIntakePower(double power);

   bool GetAlgaePhotoEye();

 private:
  
   SparkMax  m_algMotor1{ALG_MOTOR_CAN_ID, SparkMax::MotorType::kBrushless};

   frc::DigitalInput         m_algaePhotoEye     {ALGAE_PHOTO_EYE};

};
