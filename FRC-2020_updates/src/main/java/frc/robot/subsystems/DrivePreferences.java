// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import frc.robot.util.misc.PIDFController;

/*
* Preferences - allow switching between different drivers or
* settings used for tracking.  
*/

public class DrivePreferences {

  Consumer<DrivePreferences> changeCallback = null;

  // PIDS are in the SparkMax, PIDFControler is used to hold the values
  // for initializing the hardware. The PID object not run on RIO. Only set on construction in drivetrain
  public int pidSlot;
  public PIDFController rpmPID;

   // time to max outout in sparkmax (softens acceleration)
  public double slewRateLimit;      

  // limits on speeds we let the robot hit
  public double maxVelocity;
  public double maxRotRate;
  
  //future- add shifting preferences

  // accessors for user interface/tuning
  public void setMaxVelocity(double vel) { maxVelocity = vel; }
  public void setMaxRotation(double rotationRate) { maxRotRate = rotationRate; }
  public void setSlewRateLimit(double slewrate) {
    //slewrate required changes on the controller, use a callback to push values down to hardware
    slewRateLimit = slewrate;
    if (changeCallback != null) changeCallback.accept(this);
  }

  public void setChangeCallback(Consumer<DrivePreferences> callback) {
    changeCallback = callback;
  }

}