// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.ShooterWarmUp;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.ifx.ArcadeDrive;


public class auto_shooting_cmd extends SequentialCommandGroup {
  /** Creates a new auto_shooting_cmd. */

  /**
   *   Need to set this up to do the right checks at init or execute(), not construction.
   *   Intent makes sense, just not at construction.
   * 
   *  TO DO:  fix command order and exection timing.
   */

  private final Intake_Subsystem intake;
  private final Limelight_Subsystem limelight;

  public auto_shooting_cmd(Intake_Subsystem intake, ShooterSettings cmdData, ArcadeDrive drive, 
                            Limelight_Subsystem limelight, double maxSpeed) {
    
    this.intake = intake;
    this.limelight = limelight;
    addCommands(new InstantCommand(limelight::enableLED)); //turn on limelight to get target; may not be enought time?

    if (limelight.valid() && intake.getAutoShootingMode())
    { //limelight-guided auto-aim shooting mode, only auto-aim if limelight has a target, and if auto shooting mode is true 
      addCommands(new ShooterWarmUp(ShooterOnCmd.dataHigh), 
                  new auto_limelightTurnToShoot_cmd(drive, limelight, 1),
                  new Shoot(ShooterOnCmd.dataHigh));
    } else {
      //otherwise simple manual shooting
      addCommands(new Shoot(ShooterOnCmd.dataHigh));
    }
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.shooterOff();
      limelight.disableLED();
      intake.getMagazine().beltOff();
    }

}
