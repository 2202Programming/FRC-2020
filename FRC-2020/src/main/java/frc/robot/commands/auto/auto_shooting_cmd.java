// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;


public class auto_shooting_cmd extends SequentialCommandGroup {
  /** Creates a new auto_shooting_cmd. */

  public auto_shooting_cmd(Intake_Subsystem intake, ShooterOn.Data cmdData, ArcadeDrive drive, 
                            Limelight_Subsystem limelight, double maxSpeed) {

    if (intake.getShootingMode())
    { //limelight-guided auto-aim shooting mode
      addCommands(new auto_limelightTurnToShoot_cmd(drive, limelight, 1),
                  new ShooterOn(intake, ShooterOnCmd.data));
    } else {
      addCommands(new ShooterOn(intake, ShooterOnCmd.data));
    }
    
  }
}
