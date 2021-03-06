// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.ShooterWarmUp;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;


public class scoot_and_shoot extends SequentialCommandGroup {
//move to pose then shoot, either blindly or with limelight targeting

  private final Intake_Subsystem intake;
  private final Limelight_Subsystem limelight;
  private final VelocityDifferentialDrive_Subsystem drive;

  public scoot_and_shoot(Intake_Subsystem intake, VelocityDifferentialDrive_Subsystem drive, 
                            Limelight_Subsystem limelight, Pose2d endPose, double maxSpeed) {
    
    this.intake = intake;
    this.limelight = limelight;
    this.drive = drive;

    addCommands(new InstantCommand(limelight::enableLED) //enable limelight LED
                //new goToPose(drive, ) //go to Pose
               ); 

    if (limelight.valid())
    { //limelight-guided auto-aim shooting mode, only auto-aim if limelight has a target
      addCommands(new ShooterWarmUp(ShooterOnCmd.dataHigh), 
                  new auto_limelightTurnToShoot_cmd(drive, limelight, 1),
                  new Shoot(ShooterOnCmd.dataHigh));

    } else {
      //otherwise blindly shoot
      addCommands(new Shoot(ShooterOnCmd.dataHigh));
    }
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.shooterOff();
      limelight.disableLED();
      intake.getMagazine().beltOff();
      drive.tankDrive(0,0);
    }

}
