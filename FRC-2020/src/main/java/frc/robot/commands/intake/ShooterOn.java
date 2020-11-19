/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_Subsystem;

public class ShooterOn extends CommandBase {
  private double SLOW_MAG_REVERSE = -0.8; // motor power
  private double FAST_MAG_FORWARD =  1; // motor power
  private Intake_Subsystem m_intake;
  private final double m_rpmTarget_low;
  private final double m_rpmTarget_high;
  private final int m_backupCount;
  private int m_count;
  private double m_rpm;  // speed to use based on high/low mag position

  public ShooterOn(Intake_Subsystem intake, double rpmTarget_low, double rpmTarget_high, double backupSec) {
    m_intake = intake;
    m_rpmTarget_low = rpmTarget_low;
    m_rpmTarget_high = rpmTarget_high;
    m_backupCount = (int) Math.floor(backupSec / Constants.DT);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.magazineOff();
    m_intake.intakeOff();
    m_count = 0;

    // use the mag position to determine shooter speed.
    m_rpm = calcShooterSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //check our speed
    double shooterRPM =  m_intake.getShooterRPM();

    if (m_count++ < m_backupCount) {
      // We will want to backup the mag a little bit before shooter gets engaged
      // this will prevent balls getting stuck.
      m_intake.magazineOn(SLOW_MAG_REVERSE);
    } else if(shooterRPM <= m_rpm) {
      m_intake.magazineOff();
      // Runs while the shooters are getting up to their desired speed
      // This will not work if the upper and lower shooters speeds are ever 
      // changed separately
      m_intake.shooterOn(m_rpm);
    } else {
      m_intake.magazineOn(FAST_MAG_FORWARD);
    }
  }

  double calcShooterSpeed() {
    return (m_intake.isMagazineUp()) ?  m_rpmTarget_high : m_rpmTarget_low;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.shooterOff();
    m_intake.magazineOff();
    m_intake.intakeOff();
  }

}
