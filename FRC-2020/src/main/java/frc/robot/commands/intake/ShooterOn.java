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
  private final double m_upperRpmTarget_low;
  private final double m_upperRpmTarget_high;
  private final double m_lowerRpmTarget_low;
  private final double m_lowerRpmTarget_high;
  private final int m_backupCount;
  private int m_count;
  private double m_rpmUpper;  // speed to use based on high/low mag position
  private double m_rpmLower;  // speed to use based on high/low mag position
  private int stage = 0;
  private double time;


  public ShooterOn(Intake_Subsystem intake, double upperRpmTarget_low, double upperRpmTarget_high, double lowerRpmTarget_low, double lowerRpmTarget_high, double backupSec) {
    m_intake = intake;
    m_upperRpmTarget_low = upperRpmTarget_low;
    m_upperRpmTarget_high = upperRpmTarget_high;
    m_lowerRpmTarget_low = lowerRpmTarget_low;
    m_lowerRpmTarget_high = lowerRpmTarget_high;
    m_backupCount = (int) Math.floor(backupSec / Constants.DT);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.magazineOff();
    m_intake.intakeOff();
    m_count = 0;

    // use the mag position to determine shooter speed.
    m_rpmUpper = calcShooterSpeedUpper();
    m_rpmLower = calcShooterSpeedLower();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switch(stage){
      case 0:
        if(m_count++ > m_backupCount){
          stage = 1;
          time = System.currentTimeMillis();
        }
        m_intake.magazineOn(SLOW_MAG_REVERSE);
      break;
      case 1:
        if(m_intake.atGoalRPM(m_rpmUpper, m_rpmLower, .1)){
          System.out.println("Time taken in stage 1: " + (time-System.currentTimeMillis()));
          stage = 2;
        }
        m_intake.magazineOff();
        // Runs while the shooters are getting up to their desired speed
        // This will not work if the upper and lower shooters speeds are ever 
        // changed separately
        m_intake.shooterOn(m_rpmUpper, m_rpmLower);
      break;
      case 2:
        if(!m_intake.atGoalRPM(m_rpmUpper, m_rpmLower, .1)){
          stage = 1;
        }
        m_intake.magazineOn(FAST_MAG_FORWARD);
      break;
    }

    // if (m_count++ < m_backupCount) {
    //   // We will want to backup the mag a little bit before shooter gets engaged
    //   // this will prevent balls getting stuck.
    //   m_intake.magazineOn(SLOW_MAG_REVERSE);
    // } else if(!m_intake.atGoalRPM(m_rpmUpper, m_rpmLower, .1) && !trueOnce) {
    //   m_intake.magazineOff();
    //   // Runs while the shooters are getting up to their desired speed
    //   // This will not work if the upper and lower shooters speeds are ever 
    //   // changed separately
    //   m_intake.shooterOn(m_rpmUpper, m_rpmLower);
    // } else {
    //   trueOnce = true;
    //   m_intake.magazineOn(FAST_MAG_FORWARD);
    // }
  }

  double calcShooterSpeedUpper() {
    return (m_intake.isMagazineUp()) ?  m_upperRpmTarget_high : m_upperRpmTarget_low;
  }

  double calcShooterSpeedLower() {
    return (m_intake.isMagazineUp()) ?  m_lowerRpmTarget_high : m_lowerRpmTarget_low;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage = 0;
    m_intake.shooterOff();
    m_intake.magazineOff();
    m_intake.intakeOff();
  }

}
