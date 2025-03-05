#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Drive.h"
#include "subsystems/Claw.h"
#include "subsystems/Elevator.h"

class Autos {
 public:
  explicit Autos(Drive& driveSub, Claw& claw, Elevator& elevator)
      : m_driveSub{driveSub}, m_claw{claw}, m_elevator{elevator} {
    BindCommandsAndTriggers();

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::make_pair(NOTHING, frc2::cmd::None()),
        std::make_pair(TEST, pathplanner::PathPlannerAuto("Test").ToPtr()));

    autoChooser.SetDefaultOption("Do Nothing", AutoSelector::NOTHING);
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
    
     autoChooser.AddOption("Test", AutoSelector::Test);

  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {}

  enum AutoSelector {
    NOTHING,
    Test,
  };

  frc::SendableChooser<AutoSelector> autoChooser;

  Drive& m_driveSub;
  Claw& m_claw;
  Elevator& m_elevator;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
