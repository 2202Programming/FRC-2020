// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterWarmUp extends InstantCommand {
  
  final Intake_Subsystem intake;
  ShooterSettings goals;

  public ShooterWarmUp(Intake_Subsystem intake, ShooterSettings goals) {
    // Use addRequirements() here to declare subsystem dependencies.
    // not sure about requierments addRequirements(intake);
    this.intake = intake;
    this.goals = goals;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.spinupShooter(goals);
  }
}
