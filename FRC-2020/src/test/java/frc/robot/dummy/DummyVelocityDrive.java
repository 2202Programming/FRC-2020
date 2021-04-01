/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dummy;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.subsystems.DrivePreferences;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;

/**
 * 
 */
public class DummyVelocityDrive implements VelocityDrive {
  Rotation2d m_rot = new Rotation2d(90);
  Pose2d m_pose = new Pose2d(0.0, 0.0, m_rot);

  @Override
  public double getLeftVel(boolean normalized) {
    return 0;
  }

  @Override
  public double getRightVel(boolean normalized) {
    return 0;
  }

  @Override
  public double getAvgVelocity(boolean normalized) {
    return 0;
  }

  @Override
  public double getLeftPos() {
    return 0;
  }

  @Override
  public double getRightPos() {
    return 0;
  }

  @Override
  public void resetPosition() {

  }

  @Override
  public void setCoastMode(boolean coast) {
  }

  @Override
  public void velocityArcadeDrive(double lengthPerSecond, double degreePerSecond) {
  }

  @Override
  public Shifter getShifter() {
    return null;
  }

  @Override
  public Pose2d getPose() {
    return m_pose;
  }

  @Override
  public void resetOdometry(Pose2d pose) {
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return null;
  }

  @Override
  public DifferentialDriveKinematics getDriveKinematics() {
    return null;
  }

  @Override
  public void velocityTankDrive(double leftvel, double rightvel) {
  }

  @Override
  public double getMaxVelocity() {
    return 0;
  }

  @Override
  public double getMaxRotation() {
    return 0;
  }

  @Override
  public DifferentialDriveWheelSpeeds getCommandedWheelSpeeds() {
    return null;
  }

  @Override
  public void setDrivePreferences(DrivePreferences pref) {
  }

  @Override
  public DrivePreferences getDrivePreferences() {

    return null;
  }

  @Override
  public void reqShiftUp() {

  }

  @Override
  public void reqShiftDown() {

  }

  @Override
  public void setBrakeMode(boolean brakeOn) {
  }

}
