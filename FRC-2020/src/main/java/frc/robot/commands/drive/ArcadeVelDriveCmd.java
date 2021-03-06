/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.Shifter.Gear;
import frc.robot.subsystems.ifx.VelocityDrive;

public class ArcadeVelDriveCmd extends CommandBase {
  DriverControls dc;
  VelocityDrive drive;
  Shifter shifter;

  // max speeds allowed - initialized from DriverPreferences
  double vMax; // fps
  double rotMax; // deg per sec

  // AutoShift info - Change settings in RobotContainer
  double shiftUpSpeed = 6.8;    // ft/s above shift into high gear (must be less than 10 or so)
  double shiftDownSpeed = 1.5;  // ft/s below shift into low gear
  int minTimeInZone = 5;        // frame counts *.02 = 0.1 seconds
  int timeWantingUp;
  int timeWantingDown;

  int timeWantingCoast; // frames where Vcmd < Vrobot
  int minTimeEnterCoast = 3; // frames to require before coast

  // robot values measured at start of frame
  double velAvg;
  double velCmd;
  double rotCmd;

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
  public ArcadeVelDriveCmd(DriverControls dc, VelocityDrive driveTrain, Shifter shifter) {
    this.dc = dc;
    this.drive = driveTrain;
    this.shifter = shifter;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetTimeInZone();
  }

  void resetTimeInZone() {
    timeWantingDown = 0;
    timeWantingUp = 0;
  }

  void countTimeInShiftZone() {
    // use ABS of velocity
    @SuppressWarnings("unused")
    double cmd = Math.abs(velCmd);
    double vel = Math.abs(velAvg);

    // count time we want to shift high, if we hit it request it from the shifter
    if ((vel > shiftUpSpeed) && // (velCmd >= vel) &&
        (shifter.getCurrentGear() == Gear.LOW)) {
      if (++timeWantingUp >= minTimeInZone) {
        drive.reqShiftUp();
        drive.setBrakeMode(false);  // no brakes in high gear, save our carpets
        resetTimeInZone();
      }
    }

    // same thing on the low side
    if ((vel < shiftDownSpeed) && // (velCmd <= vel) &&
        (shifter.getCurrentGear() == Gear.HIGH)) {
      if (timeWantingDown++ >= minTimeInZone) {
        drive.reqShiftDown();
        resetTimeInZone();
        drive.setBrakeMode(true);
      }
    }

    checkCoastMode(cmd, vel);

  }

  @SuppressWarnings("unused")
  private void checkCoastMode(double absCmd, double absVel) {
    if (shifter.getCurrentGear() == Gear.LOW) {
      //no coast mode in low gear
      timeWantingCoast = 0;
      drive.setCoastMode(false);
      return;
    }

    // see if we can coast, using abs vel
    if ((absCmd < absVel) && (rotCmd == 0.0 )) {
      if (++timeWantingCoast > minTimeEnterCoast)
        drive.setCoastMode(true);
    } else {
      // we want more speed, clear the count
      timeWantingCoast = 0;
      drive.setCoastMode(false);
    }
  }

  /**
   * 
   * @param zoneCount - frames to wait before shifting (10-50 expected)
   * @param shiftDown - speed to downshift ft/s
   * @param shiftUp   - speed to upshift ft/s
   */
  public void setShiftProfile(int zoneCount, double shiftDown, double shiftUp) {
    this.shiftUpSpeed = shiftUp;
    this.shiftDownSpeed = shiftDown;
    this.minTimeInZone = zoneCount;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //max speed/rotation can change via 
    vMax = drive.getMaxVelocity();
    rotMax = drive.getMaxRotation();
    velAvg = drive.getAvgVelocity(false);   // ft/s

    // read controls in normalize units +/- 1.0, scale to physical units
    velCmd = dc.getVelocity() * vMax;
    rotCmd = dc.getRotation() * rotMax;
    rotCmd = applyDeadband(rotCmd, 1.0, rotMax);   // put control on this

    // implement auto-shift if it is enabled
    if (shifter.isAutoShiftEnabled()) { 
      // handles all the shift up/down requests based on time in the zone.
      countTimeInShiftZone(); 
    }
    drive.velocityArcadeDrive(velCmd, rotCmd);
  }

  // This command should never end, it can be a default command
  @Override
  public boolean isFinished() {
    return false;
  }



/**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and max.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    public static double applyDeadband(final double value, double deadband, double max) {
      if (Math.abs(value) > deadband) {
          if (value > 0.0) {
              return max * (value - deadband) / (max - deadband);
          } else {
              return max * (value + deadband) / (max - deadband);
          }
      } else {
          return 0.0;
      }
  }



}
