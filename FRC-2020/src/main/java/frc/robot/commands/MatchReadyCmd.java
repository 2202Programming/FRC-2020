// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MatchReadyCmd extends SequentialCommandGroup {
  /** Creates a new MatchReadyCmd. */
  public MatchReadyCmd() {
    RobotContainer rc = RobotContainer.getInstance();
    Intake_Subsystem intake = rc.intake;
    Magazine_Subsystem mag = intake.getMagazine();
    MagazinePositioner positioner = mag.getMagPositioner();

    //things to ready the bot for starting a match.
    addCommands(
      new InstantCommand(mag::lower),               // mag must be lowered
      new WaitCommand(2.0),                         // give it some time
      new InstantCommand(intake::raiseIntake),      // intake must be up
      new InstantCommand( () -> mag.setPC(3) ),     // most of time we start with 3 PC
      new InstantCommand( positioner::lock)         // lock it so it won't creep up
    );
  }
}
