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

    //CAN ID for non-motor devices
    public static final int PDP_CAN_ID = 0;    //this must be 0
    public static final int PCM1_CAN_ID = 1;   //default ID for PCM
    public static final int PCM2_CAN_ID = 2;   

    //Lidar
    public static final int FRONT_LEFT_LIDAR = 22;
    public static final int FRONT_RIGHT_LIDAR = 21;
    public static final double LIDAR_SAMPLE_TIME = 20; //in ms
    public static final double COLOR_SAMPLE_TIME = 100; //in ms
    public static final int LOG_REFRESH_RATE = 100; //in ms

    //Intake
    public static final int MAGAZINE_PCM_CAN_ID = PCM2_CAN_ID;
    public static final int MAGAZINE_UP_PCM = 0;
    public static final int MAGAZINE_DOWN_PCM = 1;
    public static final int MAGAZINE_PWM = 9;

    public static final int INTAKE_SPARK_PWM = 0;
    public static final int UPPER_SHOOTER_TALON_CAN = 18;
    public static final int LOWER_SHOOTER_TALON_CAN = 19;
    
    public static final int INTAKE_PCM_CAN_ID = PCM1_CAN_ID;
    public static final int INTAKE_UP_SOLENOID_PCM = 4;    
    public static final int INTAKE_DOWN_SOLENOID_PCM = 5;

    //Drivetrain
    public static final int FL_SPARKMAX_CANID = 30;
    public static final int ML_SPARKMAX_CANID = 31;
    public static final int BL_SPARKMAX_CANID = 32;
    public static final int FR_SPARKMAX_CANID = 33;
    public static final int MR_SPARKMAX_CANID = 34;
    public static final int BR_SPARKMAX_CANID = 35;

    //Gearshifter
    public static final int GEARSHIFT_PCM_CAN_ID = PCM1_CAN_ID;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;

    //Control Panel Manipulator
    public static final int PANEL_LIMIT_SWITCH_CH = 0;
    public static final int PANEL_ROTATION_CANID = 12;//placeholderA
    public static final int PCM_ID = 1;
    public static final int PANEL_PISTON_FORWARD_PCM = 2;
    public static final int PANEL_PISTON_REVERSE_PCM = 3;

    //Auto Delays - values to be adjusted later with testing
    public static final double DELAY_A = 0.0;
    public static final double DELAY_B = 3.0;
    public static final double DELAY_C = 6.0;
    
    //angle based on starting position
    public static final double ANGLE_A = 0;
    public static final double ANGLE_B = 2;
    public static final double ANGLE_C = -2.3;

    //departure angle based on starting position
    public static final double LIMELIGHT_DEPARTURE_ANGLE_A =  10;
    public static final double LIMELIGHT_DEPARTURE_ANGLE_B =  2;
    public static final double LIMELIGHT_DEPARTURE_ANGLE_C = -2.3;

    //departure angle based on starting position
    public static final double LIDAR_DEPARTURE_ANGLE_A = -18.0;
    public static final double LIDAR_DEPARTURE_ANGLE_B = 0.0;
    public static final double LIDAR_DEPARTURE_ANGLE_C = 21.0;

    //Limelight Area based on starting position
    public static final double AREA_A = 2.5;
    public static final double AREA_B = 2.9;
    public static final double AREA_C = 2.6;

    //Limelight Area of Departure based on starting possition
    public static final double DEPARTURE_AREA_A = 2.7;
    public static final double DEPARTURE_AREA_B = 2.7;
    public static final double DEPARTURE_AREA_C = 2.7;

    //camera paths
    public static final String FRONT_DRIVE_CAMERA_PATH = "/dev/video1";
    public static final String REAR_DRIVE_CAMERA_PATH = "/dev/video0";
    public static final String ARM_CAMERA_PATH = "/dev/video2";
}
