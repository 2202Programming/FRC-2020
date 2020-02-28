/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.util.misc.RateLimiter;
import frc.robot.util.misc.RateLimiter.InputModel;

public class ArcadeVelDriveCmd extends CommandBase {
  DriverControls dc;
  VelocityDrive drive;
  Shifter shifter;

  // max speeds allowed
  double vMax; // fps
  double rotMax; // deg per sec

  // AutoShift info
  double shiftUpSpeed = 6.0; // ft/s above shift into high gear
  double shiftDownSpeed = 2.5; // ft/s below shift into low gear
  int minTimeInZone = 20; // frame counts *.02 = 0.4 seconds
  int timeWantingUp;
  int timeWantingDown;

  RateLimiter rotRateLimiter;

  /**
   * Creates a new ArcadeVelDriveCmd to drive the system using physical units of
   * speed and rotaion to command the chassis. This will map DriverControls using
   * normalized stick inputs in Arcade mode.
   * 
   * @param dc         - driver controls IFX
   * @param driveTrain - drive train that supports VelocityDrive
   * @param shifter    - access to the shift controls (gear box or velocity drive)
   * @param velMaxFps  - max FPS scales normalized stick value
   * @param rotMaxDps  - rotation max, scales normalized stick value
   */
  public ArcadeVelDriveCmd(DriverControls dc, VelocityDrive driveTrain, Shifter shifter, double velMaxFps,
      double rotMaxDps) {
    this.dc = dc;
    this.drive = driveTrain;
    this.vMax = velMaxFps;
    this.rotMax = rotMaxDps;
    this.shifter = shifter;

    rotRateLimiter = new RateLimiter(DT, dc::getRotation, null,
         -rotMaxDps,
          rotMaxDps,
          -5.0, 5.0,  //rate deg/s^2
          InputModel.Position);
    rotRateLimiter.setRateGain(rotMax);
    rotRateLimiter.setForward(0.0);
    rotRateLimiter.initialize();

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

  void countTimeInShiftZone(double v) {
    double velCmd = Math.abs(v);
    // get robot velocity and use that to shift
    
    double vel = Math.abs(drive.getLeftVel(false));
    // count time we want to shift high, if we hit it request it from the shifter
    if ((vel > shiftUpSpeed) && // (velCmd >= vel) &&
        (shifter.getCurrentGear() == Gear.LOW_GEAR)) {
      if (++timeWantingUp >= minTimeInZone) {
        shifter.shiftUp();
        resetTimeInZone();
      }
    }
    // else timeWantingUp = 0;

    // same thing on the low side
    if ((vel < shiftDownSpeed) && // (velCmd <= vel) &&
        (shifter.getCurrentGear() == Gear.HIGH_GEAR)) {
      if (timeWantingDown++ >= minTimeInZone) {
        shifter.shiftDown();
        resetTimeInZone();
      }
    }
    // else timeWantingDown = 0;
  }
/**
 * 
 * @param zoneCount  - frames to wait before shifting (10-50 expected)
 * @param shiftDown  - speed to downshift ft/s
 * @param shiftUp    - speed to upshift  ft/s
 */
  public void setShiftProfile(int zoneCount, double shiftDown, double shiftUp) {
    this.shiftUpSpeed = shiftUp;
    this.shiftDownSpeed = shiftDown;
    this.minTimeInZone = zoneCount;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read controls in normalize units +/- 1.0
    double v = dc.getVelocity();
    double rot = dc.getRotation();
    rotRateLimiter.execute();
    double limRot = rotRateLimiter.get();

    // scale inputs based on max commands
    v *= vMax;
    rot *= rotMax;

    countTimeInShiftZone(v);
    drive.velocityArcadeDrive(v, limRot);
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }
}
