// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.VelocityDrive;

public class VelocityStepTest extends CommandBase {

  static final double DELAY_TIME = 2.0;
  enum State {Init, Running, Waiting, Done};

  final VelocityDrive drive;
  final double duration;
  final int repeat;
  double speed;          // speed to run at - step response test
  
  // cycle control
  double endTime;  // for current state        .
  State state;
  int count; 
   
  public VelocityStepTest(VelocityDrive drive, double speed, double duration, int repeat) {
    this.drive = drive;
    this.speed = speed;
    this.duration = duration;
    this.repeat = repeat;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.Init;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switch (state) {
      case Init: 
        startCycle();
        state = State.Running;
        break;
      case Running:
        if (Timer.getFPGATimestamp() >= endTime) {
          endCycle();
          state = State.Waiting;
        }
        break;
      case Waiting:
        if (Timer.getFPGATimestamp() >= endTime) {
          state = (count < repeat) ? State.Init : State.Done;
        }
        break;
      case Done:
      default:
        break;
      }
  }

  void startCycle() {
    drive.velocityTankDrive(speed, speed);
    endTime =  Timer.getFPGATimestamp() + duration;
  }

  void endCycle() {
    drive.velocityTankDrive(0, 0);
    endTime = Timer.getFPGATimestamp() + DELAY_TIME;
    speed = -speed;   // go backwards next time
    count++;          // cycle done
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == State.Done);
  }
}
