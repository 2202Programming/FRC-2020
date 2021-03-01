// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;

public class ShooterWarmUp extends InstantCommand {

  final Intake_Subsystem intake;
  ShooterSettings goals;

  public ShooterWarmUp() {
    this.intake = RobotContainer.getInstance().intake;
    goals = null;
  }

  public ShooterWarmUp(ShooterSettings goals) {
    this();
    this.goals = goals;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (goals != null) {
      // use what the intake has
      intake.setShooterSettings(goals);
    }
    // go ahead and spin it up at whatever settings are there
    intake.spinupShooter();
  }
}
