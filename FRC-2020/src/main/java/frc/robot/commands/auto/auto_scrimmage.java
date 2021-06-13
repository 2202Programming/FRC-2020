package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.Shoot;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class auto_scrimmage extends SequentialCommandGroup {

  // startup delay
  double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };

  public auto_scrimmage(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake) {
    double delay;
    int delayCode;

    // Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
    delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);

    delay = startDelay[delayCode];
    SmartDashboard.putNumber("Auto: Delay (secs)", delay + 3); // add 3 b/c the driveoffline takes 3 secs

    addCommands(new IntakePosition(intake, IntakePosition.Direction.Down), new WaitCommand(delay),
        new MagazineAngle(intake, 20 /* magazine angle in degrees */), new Shoot());

  }
}