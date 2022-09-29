/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import frc.robot.subsystems.Limelight_Subsystem;

/**
 * 
 * Uses Limelight to calculate angle/vel for the shooter.
 * 
 * The basic Shoot command is used, but the CalculateShooterSettings()
 * is overidden with goals.ShooterSettings calculated from limelight
 * area as an approxmation to distance.
 * 
 * Simple model for estimating angle from area
 *   y = mA + b  <angle>
 * 
 *  A = area measured from limelight
 *  m = slope <degres per Area>
 *  b =  min angle <degrees>
 * 
 */

public class ShootWithLimelight extends Shoot  {
 
  private Limelight_Subsystem m_limelight;
  private double upper_slope;
  private double upper_yIntercept;

  public ShootWithLimelight(double upper_slope, double upper_yIntercept, Limelight_Subsystem limelight)  {
    super();
    
    // new functionality for limelight
    m_limelight = limelight;
    this.upper_slope = upper_slope;
    this.upper_yIntercept = upper_yIntercept;
  
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
  public boolean calculateShooterSettings() {
     if (m_limelight.getTarget()){
          calculateRPMFromLimelight();
          //System.out.println("Got Limelight Area, Calc Upper RPM" + goals.ShooterGoal.toString() + "\n"); 
          return true;
     }
     // keep trying for target
     return false;
  }

  //find RPM/angle goals from limelight area and slope/intercept of measured RPM data
  private void calculateRPMFromLimelight(){ 
    shooterSettings.angle =  upper_slope*m_limelight.getArea() + upper_yIntercept;
    shooterSettings.rps = 10;
    shooterSettings.vel = 35;
  }
}
