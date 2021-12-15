// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ifx;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Common interfaces for the robot to tell us where it is and
 * how far it has gone.
 * 
 **/
public interface Odometry extends Subsystem {

  //Ramsete requires these
  public Pose2d getPose();
  public void resetOdometry(Pose2d pose);
  public DifferentialDriveWheelSpeeds getWheelSpeeds();
  public DifferentialDriveWheelSpeeds getCommandedWheelSpeeds();

  public DifferentialDriveKinematics getDriveKinematics();

  // Position in physical units, tracked in robot body axis
  public double getLeftPos();       // length since last reset
  public double getRightPos();      // length since last reset
  public void resetPosition();      // resets position counters

  /**
     * Gets velocity from the drive train in physical or normalized units.
     * This should represent the drive trains best estimate of current speed.
     * Units is length/s or normalized [-1 to 1].  Normalized by max speed
     * 
     * Robot axis, forward is positive.
     *  
     * @param normalized  when true, returns vel in range of -1.0 to 1.0
     * @return
     */
    public double getLeftVel(boolean normalized);
    public double getRightVel(boolean normalized);
    public double getAvgVelocity(boolean normalized);
    

}
