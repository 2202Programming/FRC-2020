/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;

public class ArcadeVelDriveCmd extends CommandBase {
  DriverControls dc;
  VelocityDrive drive;
  Shifter shifter;

  // max speeds allowed
  double vMax; // fps
  double rotMax; // deg per sec

  //AutoShift info
  double shiftUpSpeed = 6.5;    // ft/s above shift into high gear
  double shiftDownSpeed = 4.0;  // ft/s below shift into low gear
  int MinTimeInZone = 25;  // frame counts *.02 = 0.5 seconds
  int timeWantingUp;
  int timeWantingDown;

  /**
   * Creates a new ArcadeVelDriveCmd to drive the system using physical
   * units of speed and rotaion to command the chassis. This will map 
   * DriverControls using normalized stick inputs in Arcade mode.
   * 
   * @param dc           - driver controls IFX
   * @param driveTrain   - drive train that supports VelocityDrive
   * @param shifter      - access to the shift controls (gear box or velocity drive)
   * @param velMaxFps    - max FPS scales normalized stick value
   * @param rotMaxDps    - rotation max, scales normalized stick value
   */
  public ArcadeVelDriveCmd(DriverControls dc, VelocityDrive driveTrain, Shifter shifter, double velMaxFps, double rotMaxDps) {
    this.dc = dc;
    this.drive = driveTrain;
    this.vMax = velMaxFps;
    this.rotMax = rotMaxDps;
    this.shifter = shifter;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetPosition();
    shifter.shiftDown();
    resetTimeInZone();
  }

  void resetTimeInZone() {
    timeWantingDown = 0;
    timeWantingUp = 0;
  }

  void countTimeInShiftZone(double vel) {
    //count time we want to shift high, if we hit it request it from the shifter
    if ((vel > shiftUpSpeed) && (shifter.getCurrentGear() == Gear.LOW_GEAR)) {
      if (++timeWantingUp >= MinTimeInZone) {
        shifter.shiftUp();
        resetTimeInZone();
      }
    }
    // same thing on the low side
    if ((vel < shiftDownSpeed) && (shifter.getCurrentGear() == Gear.HIGH_GEAR)) {
      if (timeWantingDown++ >= MinTimeInZone) {
        shifter.shiftDown();
        resetTimeInZone();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read controls in normalize units +/- 1.0
    double v = dc.getVelocity();
    double rot = dc.getRotation();

    // scale inputs based on max commands
    v *= vMax;
    rot *= rotMax;

    countTimeInShiftZone(v);
    drive.velocityArcadeDrive(v, rot);
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }
}
