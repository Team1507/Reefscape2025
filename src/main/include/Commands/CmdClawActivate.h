#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

/**
 * @brief Command that controls the claw activation sequence.
 *
 * When the command is started, it checks whether a coral piece is already held.
 * - If not, it runs an intake sequence to grab the piece.
 * - If a piece is already held, it runs a shoot‐out sequence to eject it.
 */
class CmdClawActivate
    : public frc2::CommandHelper<frc2::Command, CmdClawActivate> {
 public:
  /**
   * Constructs the command.
   * @param power The motor power for running the claw forward (intake).
   */
  CmdClawActivate(double power);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  // State machine states for the claw operation.
  enum class ClawState {
    // Intake states
    RunClawFull,
    Sensor1,
    RunClawCreep,
    NotDetected,
    RunBackwardUntilClawSeen,
    StopMotor,
    // ShootOut states
    ShootOutStart,
    ShootOutRunning,
    ShootOutStop,
    EndState
  };

  // The current state of the state machine.
  ClawState currentState = ClawState::EndState;

  // Overall operation mode.
  enum class OperationMode {
    Intake,
    ShootOut
  };

  OperationMode m_operationMode;

 private:
  double m_power;
   frc::Timer m_timer;
};
