/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_Subsystem;

public class ShooterOn extends CommandBase {
  private double SLOW_MAG_REVERSE = -0.2; // motor power
  private double FAST_MAG_FORWARD =  0.8; // motor power
  private Intake_Subsystem m_intake;
  private final double m_rpmTarget;
  private final int m_backupCount;
  private int m_count;

  public ShooterOn(Intake_Subsystem intake, double rpmTarget, double backupSec) {
    m_intake = intake;
    m_rpmTarget = rpmTarget;
    m_backupCount = (int) Math.floor(backupSec / Constants.DT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterOn-Inited");
    m_intake.magazineOff();
    m_count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_count++; // frame counter 20ms per tick
    if (m_count < m_backupCount) {
      // We will want to backup the mag a little bit before shooter gets engaged
      // this will prevent balls getting stuck.
      m_intake.magazineOn(SLOW_MAG_REVERSE);
    } else {
      m_intake.magazineOn(FAST_MAG_FORWARD);
      m_intake.shooterOn(m_rpmTarget);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.shooterOff();
    m_intake.magazineOff();
    System.out.println("ShooterOn-Ended");
  }

}
