#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "Robot.h"
#include "Subsystems/CommandSwerveDrivetrain.h"

class Autos {
 public:
  explicit Autos(subsystems::CommandSwerveDrivetrain drivetrain)
      : m_drivetrain{drivetrain} 
      {
    BindCommandsAndTriggers();

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
   
        std::pair{TEST, pathplanner::PathPlannerAuto("Test").ToPtr()});
       
        autoChooser.AddOption("Test", AutoSelector::TEST);
   
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
 

  enum AutoSelector {
  
    TEST
 
  };

  frc::SendableChooser<AutoSelector> autoChooser;

subsystems::CommandSwerveDrivetrain& m_drivetrain;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};