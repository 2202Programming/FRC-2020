/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FRONT_LEFT_LIDAR = 21;
    public static final int FRONT_RIGHT_LIDAR = 22;
    public static final double LIDAR_SAMPLE_TIME = 100; //in ms
    public static final double COLOR_SAMPLE_TIME = 100; //in ms
    public static final int LOG_REFRESH_RATE = 100; //in ms

    //Intake
    public static final int INTAKE_UP_DIO = 4;
    public static final int INTAKE_DOWN_DIO = 4;
    public static final int INTAKE_TALON_CAN = 20;
    public static final int MAGAZINE_TALON_CAN = 21;
    public static final int UPPER_SHOOTER_TALON_CAN = 22;
    public static final int LOWER_SHOOTER_TALON_CAN = 23;
    public static final int ELEVATOR_TALON_CAN = 24;
    
    
}
