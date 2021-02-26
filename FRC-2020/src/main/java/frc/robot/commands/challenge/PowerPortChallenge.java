// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.challenge;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.ShooterWarmUp;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

public class PowerPortChallenge extends SequentialCommandGroup  {
  
  final Intake_Subsystem intake;
  final Magazine_Subsystem mag;
  final Limelight_Subsystem limelight;
  final Shoot shootCmd;

  public PowerPortChallenge(Shoot.Data shootData) { 
    this.withName("PowerPortChallenge");
    RobotContainer rc = RobotContainer.getInstance();
    intake = rc.intake;
    limelight = rc.limelight;
    mag = intake.getMagazine();

    shootCmd = new Shoot(intake, shootData);

    addCommands(
      new ShooterWarmUp(rc.intake, shootData.ShooterGoal),
      new InstantCommand(rc.limelight::enableLED)
      );
  }

  @Override
  public void initialize() {
      super.initialize();
  }

  @Override
  public void execute() {
    super.execute();

    // watch for LL to have the goal, schedule the shoot and wait for it
    if (limelight.getLEDStatus() ) {
      // should this be Proxy or normal scheduleCommand????
        new ProxyScheduleCommand(
          shootCmd ,
          new WaitUntilCommand( () -> {return (mag.getPC() == 0);} ) 
      );
      System.out.println("schedulling shoot and waitunitl");
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
