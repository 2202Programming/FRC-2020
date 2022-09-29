/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;

/**
 * Minimal controls for a two shifting the gear train.
 */
public interface Shifter {
  public enum Gear {LOW, HIGH};

    // what a shifter can do
    public void shiftDown();
    public void shiftUp();
    public Gear getCurrentGear();

    // Support autoshift controls
    public boolean isAutoShiftEnabled();   //return true if enabled
    public boolean enableAutoShift();      //return true if autoshift was set
    public boolean disableAutoShift();     //returns false

    // Tell us about the gear ratios, no state change
    public double getGearRatio(Gear g);    // gear ratio for given gear 
    public double getGearRatio();          // current gear ratio 
}
