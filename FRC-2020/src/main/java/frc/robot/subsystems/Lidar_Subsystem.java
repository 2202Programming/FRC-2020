/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ifx.*;

public class Lidar_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Lidar_Subsystem.
   */

  private TimeOfFlight front_left_lidar;
  private TimeOfFlight front_right_lidar;
  private double left_lidar_range;
  private double right_lidar_range;
  private final double LIDAR_DIST = 348;
  private double angle;
  private boolean validRange;
  private double filteredValid;
  private LinearFilter left_iir;
  private LinearFilter right_iir;
  private LinearFilter valid_fir;
  private double filterTC = 0.8;   //seconds, cutoff 1.25Hz
  private final double BUMPER_DISTANCE = 100; //mm from bumber to sensor
  public Lidar_Subsystem() {

    front_left_lidar = new TimeOfFlight(Constants.FRONT_LEFT_LIDAR);
    front_right_lidar = new TimeOfFlight(Constants.FRONT_RIGHT_LIDAR);

    front_left_lidar.setRangingMode(TimeOfFlight.RangingMode.Short, Constants.LIDAR_SAMPLE_TIME);
    front_right_lidar.setRangingMode(TimeOfFlight.RangingMode.Short, Constants.LIDAR_SAMPLE_TIME);

    // use a lowpass filter to clean up high freq noise. Helpful if you use a PID with any D.
    left_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    right_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    valid_fir = LinearFilter.movingAverage(5); //takes int taps as parameter (random)
  }

  public double getAverageRange() {
    return ((left_lidar_range+right_lidar_range)/2);
  }

  public double findAngle(){
    //getRange() returns distance in milimeters 
    double dist2 = left_lidar_range;
    double dist1 = right_lidar_range;
    double difference = dist1 - dist2;
    angle = Math.toDegrees(Math.atan(difference/LIDAR_DIST));
    return angle;
  }

  public void log() {
    SmartDashboard.putNumber("Front Left Lidar", left_lidar_range);
    SmartDashboard.putNumber("Front Right Lidar", right_lidar_range);
    SmartDashboard.putNumber("Lidar Angle", angle);
    SmartDashboard.putBoolean("Range is valid", validRange);
    SmartDashboard.putBoolean("Left lidar valid", front_left_lidar.isRangeValid());
    SmartDashboard.putBoolean("Right lidar valid", front_right_lidar.isRangeValid());

  }

  public boolean valid(){
   
    if(front_left_lidar.isRangeValid() == false || front_right_lidar.isRangeValid() == false){
      return false;
    }
    return true;
    

    //return (front_left_lidar.getRange() < 1500 && front_right_lidar.getRange() < 1500);

  }

  public boolean isEitherValid(){
    if(front_left_lidar.isRangeValid() == false && front_right_lidar.isRangeValid() == false){
      return false;
    }
    return true;
  }

  public boolean isFilteredValid(){
    return (filteredValid >= 0.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    validRange = valid();
      double temp = (validRange) ? 1.0 : 0.0;
      filteredValid = valid_fir.calculate(temp);
      left_lidar_range = left_iir.calculate(front_left_lidar.getRange())-BUMPER_DISTANCE; 
      right_lidar_range = right_iir.calculate(front_right_lidar.getRange())-BUMPER_DISTANCE;
      findAngle();
    
  }
}