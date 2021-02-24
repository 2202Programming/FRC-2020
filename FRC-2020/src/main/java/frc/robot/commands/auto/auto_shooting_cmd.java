// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.commands.intake.ShooterStartup;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;


public class auto_shooting_cmd extends SequentialCommandGroup {
  /** Creates a new auto_shooting_cmd. */

  private final Intake_Subsystem intake;
  private final Limelight_Subsystem limelight;

  public auto_shooting_cmd(Intake_Subsystem intake, ShooterOn.Data cmdData, ArcadeDrive drive, 
                            Limelight_Subsystem limelight, double maxSpeed) {
    
    this.intake = intake;
    this.limelight = limelight;

    if (intake.getShootingMode() && limelight.valid())
    { //limelight-guided auto-aim shooting mode, only auto-aim if limelight has a target
      //todo: warmup the shooter?
      addCommands(new ShooterStartup(intake, 2000), //warmup flywheels to 2000 RPM
                  new auto_limelightTurnToShoot_cmd(drive, limelight, 1),
                  new ShooterOn(intake, ShooterOnCmd.data));
    } else {
      addCommands(new ShooterOn(intake, ShooterOnCmd.data));
    }
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.shooterOff();
      limelight.disableLED();
    }

}
