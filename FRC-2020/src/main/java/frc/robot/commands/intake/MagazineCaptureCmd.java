// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;

public class MagazineCaptureCmd extends CommandBase {

  // constants
  double kMotorStrength = 0.8;
  double kFrameCount = 5;     // number of frames to run after LG opens, 20ms /frame

  Magazine_Subsystem mag;
  double frameCount;

  // states of our command
  enum State {
    WaitingForPC, MovingPC, CountFrames, MagFull
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
    frameCount = 0;
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
          state = State.CountFrames;  
          frameCount = 0;
        }
        break;

      case CountFrames:
        if (++frameCount <= kFrameCount) {
          state = (mag.isMagFull()) ? State.MagFull : State.WaitingForPC;
        }

      case MagFull:
        System.out.println("Mag full");
        // if it empties we can intake more
        if (!mag.isMagFull()) {
          state = State.WaitingForPC;
        }
      default:
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      //we got stopped, assume full
      state = State.MagFull;
    }
  }

  // This is a default command, it shouldn't end but could get pre-empted
  @Override
  public boolean isFinished() {
    return false;
  }
}
