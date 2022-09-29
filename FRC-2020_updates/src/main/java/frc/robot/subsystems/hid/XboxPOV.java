// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hid;

public enum XboxPOV {
  // POV Hat
  POV_UP(0), POV_RIGHT(90), POV_DOWN(180), POV_LEFT(270);

  public final int value;

  private XboxPOV(int initValue) {
    value = initValue;
  } 
  
  public int get() {
    return (int)value; 
  }
}
