package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.SetPowerCellCount;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.ShootWithCheck;
import frc.robot.commands.intake.ShooterWarmUp;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.ux.Dashboard;

public class auto_scrimmage extends SequentialCommandGroup {

  // startup delay
  double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };
  public static double MAG_ANGLE_1 = 37.25; //from testing
  public static double MAG_ANGLE_2 = 41.15; //from testing

  public auto_scrimmage(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
      Limelight_Subsystem limelight, Dashboard dashboard) {
    double delay;
    int delayCode;
    boolean autoMode;

    // Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
    delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);

    delay = startDelay[delayCode];
    SmartDashboard.putNumber("Auto: Delay (secs)", delay);

    // Switch 5 Auto Mode
    // False  is simple shoot - delay, then shoot from starting position, then drive forward 1/2 a foot. 
    // True assumes back bumper touching init line in line with port; shoots after delay then moves to trench to get more cells and then shoots again
    autoMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x10) >> 4 == 1) ? true : false;
    SmartDashboard.putBoolean("Auto: Mode", autoMode);

    if(!autoMode) {
      addCommands(
        new ShooterWarmUp(),
        new SetPowerCellCount(3), //assume we are starting with 3 in magazine
        new IntakePosition(intake, IntakePosition.Direction.Down), 
        new MagazineAngle(intake, MAG_ANGLE_1 /* magazine angle in degrees */), 
        new WaitCommand(delay), //delay to make sure we don't interact with alliance during shooting high goal
        new ShootWithCheck().withTimeout(7),
        new followTrajectory(drive, dashboard.getTrajectory("DriveOffLine"))  //drive off line half a foot for points
        );
    } 
    else {
      addCommands(
        new ShooterWarmUp(),
        new SetPowerCellCount(3), //assume we are starting with 3 in magazine
        new IntakePosition(intake, IntakePosition.Direction.Down), 
        new MagazineAngle(intake, MAG_ANGLE_1 /* magazine angle in degrees */),
        new WaitCommand(delay), //delay to make sure we don't interact with alliance during shooting high goal
        new ShootWithCheck().withTimeout(5), //assuming a straight shot in initial position
        new ParallelCommandGroup(
          new IntakePower(intake, Power.Toggle, 0.5), 
          new InstantCommand(limelight::enableLED),
          new followTrajectory(drive, dashboard.getTrajectory("GoToTrenchFromCenter"))
        ), //go to trench, get 3 powercells
        new MagazineAngle(intake, MAG_ANGLE_2 /* magazine angle in degrees */),
        //new auto_limelightTurnToShoot_cmd(drive, limelight, 1 /* speed */).withTimeout(1), 
        new Shoot().withTimeout(5),
        new InstantCommand(limelight::disableLED)
        );
    }
  }
}