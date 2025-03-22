#include "Commands/CmdPivotManual.h"
#include "Robot.h"
#include "Constants/Constants.h"
#include <iostream>
#include <cmath>
#include <units/angle.h>

#define PIVOT_DEADBAND 0.1
// Adjust this scale factor to match the range of your mechanism in turns.
// For example, if a full joystick deflection should move the pivot 1 turn, set it to 1.0.
#define MANUAL_POSITION_SCALE 0.3

CmdPivotManual::CmdPivotManual() {
  // Declare dependency on the pivot subsystem.
  AddRequirements(&robotcontainer.m_pivot);
}

void CmdPivotManual::Initialize() {
  std::cout << "Starting Pivot Manual Motion Magic" << std::endl;
}

void CmdPivotManual::Execute() {
  // // Get joystick value (left Y axis).
  // double joystickValue = robotcontainer.m_topDriver.GetLeftY();
  
  // // Apply deadband.
  // if (std::fabs(joystickValue) < PIVOT_DEADBAND) {
  //   joystickValue = 0;
  // }
  
  // // Convert joystick input to a target position in turns.
  
  // auto targetPosition = joystickValue * MANUAL_POSITION_SCALE;
  
  // // Command the pivot to move using Motion Magic.
  // robotcontainer.m_pivot.SetManualTarget(targetPosition);

    if((robotcontainer.m_topDriver.GetRightY() < -PIVOT_DEADBAND))
  {
    robotcontainer.m_pivot.SetPivotCoast();
    robotcontainer.m_pivot.SetPower(-0.25);
    m_manualPivotEnabled = true;
  }
  else if(robotcontainer.m_topDriver.GetRightY() > PIVOT_DEADBAND)
  {
    robotcontainer.m_pivot.SetPivotCoast();
    robotcontainer.m_pivot.SetPower(0.1);
    m_manualPivotEnabled = true;
  }
  else if(m_manualPivotEnabled)
  {
    robotcontainer.m_pivot.SetPivotBrake();
    robotcontainer.m_pivot.SetPower(-0.01);
    m_manualPivotEnabled = false;
  }
}

void CmdPivotManual::End(bool interrupted) {
  std::cout << "End Pivot Manual Motion Magic" << std::endl;
}

bool CmdPivotManual::IsFinished() {
  return false;  // This command runs until canceled.
}
