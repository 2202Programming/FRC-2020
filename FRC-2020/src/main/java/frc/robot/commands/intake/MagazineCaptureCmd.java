// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;

public class MagazineCaptureCmd extends CommandBase {

  // constants
  double kMotorStrength = 0.8;

  Magazine_Subsystem mag;

  // states of our command
  enum State {
    WaitingForPC, MovingPC, MagFull
  };

  State state = State.WaitingForPC;

  /** Creates a new MagazineCaptureCmd. */
  public MagazineCaptureCmd(Magazine_Subsystem mag) {
    this.mag = mag;
    addRequirements(mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.WaitingForPC;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (state) {
      case WaitingForPC:

        if (mag.isGateBlocked()) {
          mag.beltOn(kMotorStrength);
          state = State.MovingPC;
        }
        break;
      case MovingPC:
        if (!mag.isGateBlocked()) {
          mag.beltOff();
          mag.addPC();
          state = (mag.isMagFull()) ? State.MagFull : State.WaitingForPC;
        }
        break;

      case MagFull:
      default:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == State.MagFull);
  }
}
