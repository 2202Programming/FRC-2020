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
  private TimeOfFlight front_right_lidar;
  private double left_lidar_range;
  private double right_lidar_range;
  private long lastLidarTime;

  public Lidar_Subsystem() {

    lastLidarTime = System.currentTimeMillis();

    front_left_lidar = new TimeOfFlight(Constants.FRONT_LEFT_LIDAR);
    front_right_lidar = new TimeOfFlight(Constants.FRONT_RIGHT_LIDAR);

    front_left_lidar.setRangingMode(TimeOfFlight.RangingMode.Medium, Constants.LIDAR_SAMPLE_TIME);
    front_right_lidar.setRangingMode(TimeOfFlight.RangingMode.Medium, Constants.LIDAR_SAMPLE_TIME);

  }

  public double getAverageRange() {
    return ((left_lidar_range+right_lidar_range)/2);
  }
  public void updateLidar(){
    if ((lastLidarTime + Constants.LIDAR_SAMPLE_TIME + 10) < System.currentTimeMillis()){ //dont check lidar faster than it's sample rate, with 10ms buffer
      left_lidar_range = front_left_lidar.getRange();
      right_lidar_range = front_right_lidar.getRange();
      lastLidarTime = System.currentTimeMillis();
    }
  }

  public void printLog() {
    SmartDashboard.putNumber("Front Left Lidar", left_lidar_range);
    SmartDashboard.putNumber("Front Right Lidar", right_lidar_range);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
