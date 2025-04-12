#pragma once

#include "constants/Constants.h"
#include "constants/Presets.h"

#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class Claw : public frc2::SubsystemBase {
 public:
  Claw();
  
  void Periodic() override;

/// --- CLAW ---
  void SetClawPower(double power);
  void StopClawPower(double power);

  bool GetClawPhotoEyeFirst(void);

  bool IsCoralReady();
  
  bool m_clawStop = false;
  
  bool isCoralReady;
  
 private:
  SparkMax  m_claw{CLAW_CAN_ID, SparkMax::MotorType::kBrushed};





  frc::DigitalInput         m_armPhotoeyeFirst  {CLAW_PHOTO_EYE_FIRST};
  // frc::DigitalInput         m_algaePhotoEye     {ALGAE_PHOTO_EYE};

  

  
};