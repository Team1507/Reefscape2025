#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>
#include "frc/PowerDistribution.h"

#include "RobotContainer.h"

extern RobotContainer robotcontainer; // <--- global variable

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  //void SimulationPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  //void UpdateVision();

  frc2::Command* m_autonomousCommand = nullptr;

  //RobotContainer m_container;
  frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};

  units::second_t lastTotalLoopTime;
  nt::DoublePublisher loopTimePub{nt::NetworkTableInstance::GetDefault()
                                      .GetTable("Metadata")
                                      ->GetDoubleTopic("RobotPeriodicLoopRate")
                                      .Publish()};
  nt::DoublePublisher matchTimePub{nt::NetworkTableInstance::GetDefault()
                                       .GetTable("Metadata")
                                       ->GetDoubleTopic("MatchTime")
                                       .Publish()};
  nt::DoublePublisher battVoltagePub{nt::NetworkTableInstance::GetDefault()
                                         .GetTable("Metadata")
                                         ->GetDoubleTopic("BatteryVoltage")
                                         .Publish()};
};
