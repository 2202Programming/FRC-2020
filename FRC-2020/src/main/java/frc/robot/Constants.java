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

    public static final double DT = 0.02;  //20ms framerate 50Hz
    public static final double Tperiod = 0.02;   //framerate period 20ms, 50Hz
    
    //Lidar
    public static final int FRONT_LEFT_LIDAR = 21;
    public static final int FRONT_RIGHT_LIDAR = 22;
    public static final double LIDAR_SAMPLE_TIME = 100; //in ms
    public static final double COLOR_SAMPLE_TIME = 100; //in ms
    public static final int LOG_REFRESH_RATE = 100; //in ms

    //Intake
    public static final int INTAKE_UP_DIO = 4;
    public static final int INTAKE_DOWN_DIO = 5;
    public static final int INTAKE_SPARK_PWM = 0;
    public static final int MAGAZINE_PWM = 9;
    public static final int UPPER_SHOOTER_TALON_CAN = 18;
    public static final int LOWER_SHOOTER_TALON_CAN = 23;
    public static final int ELEVATOR_TALON_CAN = 24;
    public static final int ELEVATOR_PCM_CAN_ID = 2;
    public static final int ELEVATOR_UP_SOLENOID_PCM = 4;    
    public static final int ELEVATOR_DOWN_SOLENOID_PCM = 5;

    //Drivetrain
    public static final int FL_SPARKMAX_CANID = 30;
    public static final int ML_SPARKMAX_CANID = 31;
    public static final int BL_SPARKMAX_CANID = 32;
    public static final int FR_SPARKMAX_CANID = 33;
    public static final int MR_SPARKMAX_CANID = 34;
    public static final int BR_SPARKMAX_CANID = 35;

    //Gearshifter
    public static final int GEARSHIFT_PCM_CAN_ID = 2;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;

    //Auto Delays - values to be adjusted later with testing
    public static final double DELAY_A = 0.0;
    public static final double DELAY_B = 1.0;
    public static final double DELAY_C = 2.0;

    //camera paths
    public static final String FRONT_DRIVE_CAMERA_PATH = "/dev/video1";
    public static final String REAR_DRIVE_CAMERA_PATH = "/dev/video0";
    public static final String ARM_CAMERA_PATH = "/dev/video2";
}
