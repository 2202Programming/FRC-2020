/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double Tperiod = 0.02;   //framerate period 20ms, 50Hz

    public static final int FRONT_LEFT_LIDAR = 40;
    public static final int FRONT_RIGHT_LIDAR = 41;
    public static final double LIDAR_SAMPLE_TIME = 24; // in ms
    public static final double COLOR_SAMPLE_TIME = 100; // in ms
    public static final int LOG_REFRESH_RATE = 100; // in ms

    public static final int FL_SPARKMAX_CANID = 11;
    public static final int ML_SPARKMAX_CANID = 12;
    public static final int BL_SPARKMAX_CANID = 41;
    public static final int FR_SPARKMAX_CANID = 24;
    public static final int MR_SPARKMAX_CANID = 22;
    public static final int BR_SPARKMAX_CANID = 23;

    public static final int GEARSHIFT_PCM_ID = 1;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;
}
