/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lidar_Subsystem extends SubsystemBase {
  /**
   * Creates a new Lidar_Subsystem.
   */

  private TimeOfFlight front_left_lidar;

  public Lidar_Subsystem() {

    front_left_lidar = new TimeOfFlight(Constants.FRONT_LEFT_LIDAR);

  }

  public void log() {
    SmartDashboard.putNumber("Front Left Lidar", front_left_lidar.getRange());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
