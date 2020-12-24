/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

/**
 * enum for Xbox analog inputs (Axis)
 * 
 * 2020/12/23 DPL split from combined enum for strong type
 * 
 */
public enum XboxAxis {
    // analog triggers aka Axis
    LEFT_X(0), LEFT_Y(1), TRIGGER_LEFT(2), // left side
    TRIGGER_RIGHT(3), RIGHT_X(4), RIGHT_Y(5); // right side

    public final int value;

    private XboxAxis(int initValue) {
        value = initValue;
    }

    public int getCode() {
        return value;
    }
}