// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

/**
 * MagCaptureCmd - used as default command to drive belt while capturing 
 *     power cells.  Will count the cells and stop intake if we hit max.
 */
public class MagazineCaptureCmd extends CommandBase {

  // constants tuned to move power cells smoothly and clear light gate
  final double kMotorStrength = 0.7;
  final int kFrameCount = 6;     // number of frames to run after LG opens, 20ms /frame

  Intake_Subsystem intake;
  Magazine_Subsystem mag;
  int frameCount;

  // states of our command
  enum State {
    WaitingForPC, MovingPC, CountFrames, MagFull
  };

  State state = State.WaitingForPC;

  /** Creates a new MagazineCaptureCmd. */
  public MagazineCaptureCmd(Intake_Subsystem intake) {
    this.mag = intake.getMagazine();
    this.intake = intake;
    addRequirements(mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     state = State.WaitingForPC;
    frameCount = 0;
    if (mag.isMagFull()) {
      state = State.MagFull;
    }
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
          mag.addPC();
          state = State.CountFrames;  
          frameCount = 0;
        }
        break;

      case CountFrames:
        if (++frameCount > kFrameCount) { 
          mag.beltOff();
          state = (mag.isMagFull()) ? State.MagFull : State.WaitingForPC;
        }
        break;
      case MagFull:
        // make sure we don't pick up more than we should
        intake.intakeOff();

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
      
    }
  }

  // This is a default command, it shouldn't end but could get pre-empted
  @Override
  public boolean isFinished() {
    return false;
  }
}
