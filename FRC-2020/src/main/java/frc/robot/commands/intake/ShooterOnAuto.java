/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;

public class ShooterOnAuto extends ShooterOn  {
 
  private double m_rpmUpper;  // speed to use based on high/low mag position
  private double m_rpmLower;  // speed to use based on high/low mag position
  private Limelight_Subsystem m_limelight;
  private double upper_slope;
  private double upper_yIntercept;
  private double lower_slope;
  private double lower_yIntercept;

  public ShooterOnAuto(Intake_Subsystem intake, double backupSec, double upper_slope, double upper_yIntercept, double lower_slope, double lower_yIntercept, Limelight_Subsystem limelight)  {
    super(intake, backupSec);
    
    // new functionality for limelight
    m_limelight = limelight;
    this.upper_slope = upper_slope;
    this.upper_yIntercept = upper_yIntercept;
    this.lower_slope = lower_slope;
    this.lower_yIntercept = lower_yIntercept;
  }

  /**
   * calculateShooterSpeed() 
   *  
   *  overides the simple single value targets with calculation from limelight
   *  and maybe other estimates of distance.
   * 
   * @return  true when we are ready to shoot
   *          false - keep calling this function every frame, shooter will wait
   */
  @Override
  public boolean calculateShooterSpeed() {
    
     if (m_limelight.getTarget()){
          calculateRPMFromLimelight();
          m_intake.shooterOn(m_rpmUpper, m_rpmLower); //Start shooter spin up to actual RPM goals
          System.out.println("Got Limelight Area, Calc Upper RPM Goal = " + m_rpmUpper + ", Lower RPM Goal = " + m_rpmLower + "\n"); 
          return true;
     }
     // keep trying for target
     return false;
  }

  //find RPM goals from limelight area and slope/intercept of measured RPM data
  private void calculateRPMFromLimelight(){ 
    m_rpmUpper = upper_slope*m_limelight.getArea() + upper_yIntercept;
    m_rpmLower = lower_slope*m_limelight.getArea() + lower_yIntercept;
  }
}
