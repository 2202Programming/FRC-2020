/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ifx.*;

public class Lidar_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Lidar_Subsystem.
   */
  
  //Geometry contstants
  private final double LIDAR_DIST = RobotPhysical.LIDAR_TO_LIDAR; //mm 
  private final double BUMPER_DISTANCE = RobotPhysical.BUMPER_TO_LIDAR; //mm from bumber to sensor

  // CAN bus TLIDAR devices
  private TimeOfFlight front_left_lidar;
  private TimeOfFlight front_right_lidar;
  
  // raw ranges and validity, measured each periodic() frame, range in mm
  double left_raw = 0.0;   
  double right_raw = 0.0;
  boolean left_valid = false;
  boolean right_valid = false;
  
  // measured, filtered, and updated values
  private double left_lidar_range;
  private double right_lidar_range;
  private double angle;
  private double filteredValid;

  // filters to clean up noise, low pass filter
  private LinearFilter left_iir;
  private LinearFilter right_iir;
  private LinearFilter valid_fir;
  private double filterTC = 0.8;   //seconds, cutoff 1.25Hz
  

  private boolean isreal; //if simulation mode, do not construct lidar and all methods return 0 or nothing.
  
  public Lidar_Subsystem() {
    this.isreal = RobotBase.isReal();
    SendableRegistry.setName(this, "LIDAR");

    if (isreal){
      front_left_lidar = new TimeOfFlight(CAN.FRONT_LEFT_LIDAR);
      front_right_lidar = new TimeOfFlight(CAN.FRONT_RIGHT_LIDAR);
  
      front_left_lidar.setRangingMode(TimeOfFlight.RangingMode.Short, LIDAR.SAMPLE_mS);
      front_right_lidar.setRangingMode(TimeOfFlight.RangingMode.Short, LIDAR.SAMPLE_mS);
    }
  
    // use a lowpass filter to clean up high freq noise. Helpful if you use a PID with any D.
    left_iir = LinearFilter.singlePoleIIR(filterTC, Tperiod);
    right_iir = LinearFilter.singlePoleIIR(filterTC, Tperiod);
    valid_fir = LinearFilter.movingAverage(5); //takes int taps as parameter (random)  
  }

  public double getAverageRange() {
    return ((left_lidar_range+right_lidar_range)/2);
  }
/**
 * findAngle()
 *  Calculates the angle the robot relative to the wall or whatever is blocking the LIDAR.
 *     Positive angle is CW
 *     Negative angle is CCW
 * 
 * @return angle (degrees)
 */
  public double findAngle(){
    //getRange() returns distance in milimeters 
    double dist2 = left_lidar_range;
    double dist1 = right_lidar_range;
    double difference = dist1 - dist2;
    //DPL - should this asin(), not atan() small angles << 15 deg doesn't matter - 1/1/21
    angle = Math.toDegrees(Math.atan(difference/LIDAR_DIST));
    return angle;
  }

  public void log() {
    /**  dashboard rework - dpl 1/1/2021
    SmartDashboard.putNumber("/LIDAR/left", left_lidar_range);
    SmartDashboard.putNumber("/LIDAR/right", right_lidar_range);
    SmartDashboard.putBoolean("/LIDAR/valid", valid());
    SmartDashboard.putBoolean("/LIDAR/l-valid", left_valid);
    SmartDashboard.putBoolean("/LIDAR/r-valid", right_valid);
    SmartDashboard.putNumber("/LIDAR/angle", angle);
    */
  }

  public boolean valid() {
    return (left_valid && right_valid);
  }

  public boolean isEitherValid() {
    return (left_valid || right_valid);
  }

  public boolean isFilteredValid() {
    return (filteredValid >= 0.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read sensor and valid flags, skip if simulated
    if (isreal) {
      left_raw = front_left_lidar.getRange();
      right_raw = front_right_lidar.getRange();
      left_valid = front_left_lidar.isRangeValid();
      right_valid = front_right_lidar.isRangeValid();
    }
    else {
      // make something up
      left_raw = (left_raw + 1) % 200;
      right_raw = (right_raw + 1.03) % 200;
      right_valid = (right_raw <100);
      left_valid = (left_raw < 125);
    }
    double temp = (valid()) ? 1.0 : 0.0;
    filteredValid = valid_fir.calculate(temp);
    left_lidar_range = left_iir.calculate(left_raw) - BUMPER_DISTANCE; 
    right_lidar_range = right_iir.calculate(right_raw) - BUMPER_DISTANCE;
    findAngle(); 
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.addNumber("LDR/left",  () -> left_lidar_range).withSize(2,1);
    layout.addNumber("LDR/right", () -> right_lidar_range);
		layout.addBoolean("LDR/valid", this::isFilteredValid);
	}

}
