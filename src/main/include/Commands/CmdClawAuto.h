#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

/**
 * @brief Command that controls the ClawAutoAuto activation sequence.
 *
 * When the command is started, it checks whether a coral piece is already held.
 * - If not, it runs an intake sequence to grab the piece.
 * - If a piece is already held, it runs a shoot‐out sequence to eject it.
 */
class CmdClawAuto
    : public frc2::CommandHelper<frc2::Command, CmdClawAuto> {
 public:
  /**
   * Constructs the command.
   * @param power The motor power for running the ClawAuto forward (intake).
   */
  CmdClawAuto(double power);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  // State machine states for the ClawAutoAuto operation.
  enum class ClawAutoState {
    // Intake states
    RunClawFull,
    Sensor1,
    RunClawCreep,
    NotDetected,
    StopMotor,
    EndState
  };

  // The current state of the state machine.
  ClawAutoState currentState = ClawAutoState::EndState;

  // Overall operation mode.
  enum class OperationMode {Intake};

  OperationMode m_operationMode;

 private:
  double m_power;
   frc::Timer m_timer;
};
