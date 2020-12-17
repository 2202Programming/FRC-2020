/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;

public class ShooterOnAuto extends CommandBase {
  private double SLOW_MAG_REVERSE = -0.8; // motor power
  private double FAST_MAG_FORWARD =  1; // motor power
  private Intake_Subsystem m_intake;
  private final double m_upperRpmTarget_low = 0;
  private final double m_upperRpmTarget_high = 0;
  private final double m_lowerRpmTarget_low = 0;
  private final double m_lowerRpmTarget_high = 0;
  private final int m_backupCount;
  private int m_count;
  private double m_rpmUpper;  // speed to use based on high/low mag position
  private double m_rpmLower;  // speed to use based on high/low mag position
  private int stage = -1;
  private double time;
  private Limelight_Subsystem m_limelight;
  private double upper_slope;
  private double upper_yIntercept;
  private double lower_slope;
  private double lower_yIntercept;

  public ShooterOnAuto(Intake_Subsystem intake, double backupSec, double upper_slope, double upper_yIntercept, double lower_slope, double lower_yIntercept, Limelight_Subsystem limelight) {
    m_intake = intake;
    m_limelight = limelight;
    this.upper_slope = upper_slope;
    this.upper_yIntercept = upper_yIntercept;
    this.lower_slope = lower_slope;
    this.lower_yIntercept = lower_yIntercept;

    m_backupCount = (int) Math.floor(backupSec / Constants.DT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.magazineOff();
    m_intake.intakeOff();
    m_count = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switch(stage){
      case -1:
        m_intake.magazineOn(SLOW_MAG_REVERSE);
        m_limelight.enableLED();
        stage = 0;
      break;

      case 0: //stage 0, backup magazine for backup_count to get balls off flywheels
        if(m_count++ > m_backupCount){
          stage = 1;
          time = System.currentTimeMillis();
          m_intake.magazineOff();
          m_intake.shooterOn(1000, 1000); //shooter starts spinning at low speed, exact RPM target unknown
        }
      break;

      case 1: //waiting for limelight to get target
        if(m_limelight.getTarget()){
          calculateRPMFromLimelight();
          m_intake.shooterOn(m_rpmUpper, m_rpmLower); //Start shooter spin up to actual RPM goals
          System.out.println("Got Limelight Area, Calc Upper RPM Goal = " + m_rpmUpper + ", Lower RPM Goal = " + m_rpmLower + "\n"); 
          stage = 2;
        }
      break;

      case 2: //stage 2, pause magazine while shooters get to RPM goal
        m_count++;
        if (m_count%2 == 0) { //wait 2 robot cycles before checking RPMs again
          if(m_intake.atGoalRPM(m_rpmUpper, m_rpmLower, .05)){
            String output = "*** MAG RESUME - Shooter Ramp-up Delay: " + (System.currentTimeMillis() - time) + " ms. \n";
            output = output + "Upper Goal: " + m_intake.upperRPM_target + ", Achieved Upper RPM: " + m_intake.upperRPM + "\n";
            output = output + "Lower Goal: " + m_intake.lowerRPM_target + ", Achieved Lower RPM: " + m_intake.lowerRPM + "\n\n";
            System.out.println(output);
            stage = 3;
            m_intake.magazineOn(FAST_MAG_FORWARD);
          }
         }
      break;

      case 3: //stage 3, magazine forward fast to shoot while at RPM goals
      m_count++;
      if (m_count%2 == 0) { //wait 2 robot cycles before checking RPMs again
        if(!m_intake.atGoalRPM(m_rpmUpper, m_rpmLower, .05)){ //back to stage 2 if RPMs fall below tolerance
          stage = 2;
          time = System.currentTimeMillis();
          m_intake.magazineOff();
          System.out.println("**MAG PAUSE - Upper RPM: " + m_intake.upperRPM + ", Lower RPM: " + m_intake.lowerRPM + "\n");
        }
      }
      break;
    }
  }

  double calcShooterSpeedUpper() {
    return (m_intake.isMagazineUp()) ?  m_upperRpmTarget_high : m_upperRpmTarget_low;
  }

  double calcShooterSpeedLower() {
    return (m_intake.isMagazineUp()) ?  m_lowerRpmTarget_high : m_lowerRpmTarget_low;
  }

  private void calculateRPMFromLimelight(){ //find RPM goals from limelight area and slope/intercept of measured RPM data
    m_rpmUpper = upper_slope*m_limelight.getArea() + upper_yIntercept;
    m_rpmLower = lower_slope*m_limelight.getArea() + lower_yIntercept;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage = -1;
    m_intake.shooterOff();
    m_intake.magazineOff();
    m_intake.intakeOff();
    m_limelight.disableLED();
  }

}
