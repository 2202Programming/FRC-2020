/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ifx.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Limelight_Subsystem.
   */

   private NetworkTable table;
   private double x;
   private double filteredX;
   private double y;
   private double area; //area is between 0 and 100. Calculated as a percentage of image
   private boolean target;
   private boolean ledStatus; //true = ON
   
   private LinearFilter x_iir;
   private double filterTC = 0.8;   //seconds, cutoff 1.25Hz

  public Limelight_Subsystem() {
    disableLED();
    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); //-27 degrees to 27 degrees
    NetworkTableEntry ty = table.getEntry("ty"); // -20.5 to 20.5 degrees
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv"); //target validity (1 or 0)
    NetworkTableEntry leds = table.getEntry("ledMode"); 

    //updates global variables

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = (tv.getDouble(0)==0) ? (false) : (true);
    filteredX = x_iir.calculate(x);
    ledStatus = (leds.getDouble(0)==3) ? (true) : (false);

  }

  public double getX(){
    return x;
  }

  public double getFilteredX(){
    return filteredX;
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
}

public void enableLED() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
}

public boolean valid(){
  return target;
}

  public void log(){

      SmartDashboard.putNumber("X value", x);
      SmartDashboard.putNumber("Filtered X value", filteredX);
      SmartDashboard.putNumber("Y value", y);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putBoolean("Limelight Valid", target);

  }
}
