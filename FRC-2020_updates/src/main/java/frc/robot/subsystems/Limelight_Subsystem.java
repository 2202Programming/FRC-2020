/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;

public class Limelight_Subsystem extends MonitoredSubsystemBase implements Logger {
  /**
   * Creates a new Limelight_Subsystem.
   */

   private NetworkTable table;
   private NetworkTableEntry tx;
   private NetworkTableEntry ty;
   private NetworkTableEntry ta;
   private NetworkTableEntry tv;
   private NetworkTableEntry leds;
   private NetworkTableEntry booleanLeds;

   private double x;
   private double filteredX;
   private double y;
   private double area; //area is between 0 and 100. Calculated as a percentage of image
   private boolean target;
   private boolean ledStatus; //true = ON
   private double filteredArea;
   
   private LinearFilter x_iir;
   private LinearFilter area_iir;

   private double filterTC = 0.8;   //seconds, cutoff 1.25Hz

  public Limelight_Subsystem() {
   
    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); //-27 degrees to 27 degrees
    ty = table.getEntry("ty"); // -20.5 to 20.5 degrees
    ta = table.getEntry("ta");
    tv = table.getEntry("tv"); //target validity (1 or 0)
    leds = table.getEntry("ledMode"); 
    booleanLeds = table.getEntry("booleanLeds");  
    disableLED();
  }

  @Override
  public void monitored_periodic() {
    //updates global variables
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = (tv.getDouble(0)==0) ? (false) : (true);
    filteredX = x_iir.calculate(x);
    filteredArea = area_iir.calculate(area);
    ledStatus = (leds.getDouble(0)==3) ? (true) : (false);

  }

  public double getX(){
    return x;
  }

  public double getFilteredX(){
    return filteredX;
  }

  public double getFilteredArea(){
    return filteredArea;
  }

  public double getY(){
    return y;
  }

  public double getArea(){
    return area;
  }

  public boolean getTarget(){
    return target;
  }
  
  public boolean getLEDStatus() {
    return ledStatus;
  }

  public void disableLED() {
    leds.setNumber(1);
    ledStatus = false;
    booleanLeds.setBoolean(ledStatus);
}

public void enableLED() {
    leds.setNumber(3);
    ledStatus = true;
    booleanLeds.setBoolean(ledStatus);
}

public boolean valid(){
  return target;
}

  public void log(){

  }
}
