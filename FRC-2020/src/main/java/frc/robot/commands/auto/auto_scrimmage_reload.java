package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.Shoot;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.ux.Dashboard;

public class auto_scrimmage_reload extends SequentialCommandGroup {

  // startup delay
  double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };

  public static double MAG_ANGLE_1 = 20;
  public static double MAG_ANGLE_2 = 15;

  public auto_scrimmage_reload(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
      Limelight_Subsystem limelight, Dashboard dashboard) {
    double delay;
    int delayCode;

    // Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
    delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);

    delay = startDelay[delayCode];
    SmartDashboard.putNumber("Auto: Delay (secs)", delay + 3); // add 3 b/c the driveoffline takes 3 secs

    addCommands(new IntakePosition(intake, IntakePosition.Direction.Down), new WaitCommand(delay),
        new MagazineAngle(intake, MAG_ANGLE_1 /* magazine angle in degrees */),
        new auto_limelightTurnToShoot_cmd(drive, limelight, 1 /* speed */), new Shoot(),
        new IntakePower(intake, Power.Toggle, 0.5), new followTrajectory(drive, dashboard.getTrajectoryChooser()),
        new MagazineAngle(intake, MAG_ANGLE_2 /* magazine angle in degrees */),
        new auto_limelightTurnToShoot_cmd(drive, limelight, 1 /* speed */), new Shoot());
  }
}