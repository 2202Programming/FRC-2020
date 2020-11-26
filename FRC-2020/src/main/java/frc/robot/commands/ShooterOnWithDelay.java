/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;

public class ShooterOnWithDelay extends CommandBase {
  private Intake_Subsystem m_intake;
  private final double shootPower = 1;
  private double delay; //in milliseconds
  private double startTime;

  public ShooterOnWithDelay(Intake_Subsystem m_intake, double delay) {
    this.m_intake = m_intake;
    this.delay = delay;

    // Use addRequirements() here to declare subsystem dependencies.
   // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterOn-Inited");
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // We will want to backup the mag a little bit before shooter gets engaged
    // this will prevent balls getting stuck.

    m_intake.shooterOn(shootPower,shootPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.shooterOff();
    System.out.println("ShooterOn-Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (System.currentTimeMillis() - startTime) >= delay;
  }
}
