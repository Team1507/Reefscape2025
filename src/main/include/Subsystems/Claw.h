#pragma once

#include "constants/Constants.h"

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

/// --- ALGAE INTAKE ---
  void SetAlgaePower(double power);

  double GetAlgaePower();

  float GetAlgaeCurrent();
  float GetAlgaeTemp();

  bool GetAlgaePhotoEye();

 // --- ALGAE PIVOT ---
  void SetPosition(double position);
  void   SetPower(double power);

  double GetPosition();
  
  void ZeroEncoder(void);

  float GetPivotCurrent();
  float GetPivotTemp();

  bool IsBallLoaded() const;
  void SetBallLoaded(bool loaded);
  
  bool m_ballLoaded = false;
  
  bool isCoralReady;
  
 private:
  SparkMax  m_claw{CLAW_CAN_ID, SparkMax::MotorType::kBrushed};

  SparkMax  m_pivot{PIVOT_CAN_ID, SparkMax::MotorType::kBrushed};

  rev::spark::SparkRelativeEncoder m_pivotEncoder = m_pivot.GetEncoder();
  rev::spark::SparkClosedLoopController m_pivotPID = m_pivot.GetClosedLoopController();

  frc::DigitalInput         m_armPhotoeyeFirst  {CLAW_PHOTO_EYE_FIRST};
  frc::DigitalInput         m_algaePhotoEye     {ALGAE_PHOTO_EYE};

  
};