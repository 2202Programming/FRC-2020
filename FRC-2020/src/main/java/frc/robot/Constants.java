/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.intake.ShooterOn;
import frc.robot.subsystems.Intake_Subsystem.FlywheelRPM;
import frc.robot.util.misc.PIDFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * Conventions xxx_CANID - CAN device, integer between 0..63, unique yyy_PCM -
 * Pnumatics channel ID zzz_PWM - PWM port on RIO
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

    /**
     * CAN bus IDs
     * 
     * Please keep in order ID order
     * 
     */
    public static final class CAN {
        // CAN ID for non-motor devices
        public static final int PDP = 0; // this must be 0
        public static final int PCM1 = 1; // default ID for PCM
        public static final int PCM2 = 2;

        // Lidar 
        public static final int FRONT_RIGHT_LIDAR = 21;
        public static final int FRONT_LEFT_LIDAR = 22;
       
        // Drivetrain
        public static final int FL_SMAX = 30;
        public static final int ML_SMAX = 31;
        public static final int BL_SMAX = 32;
        public static final int FR_SMAX = 33;
        public static final int MR_SMAX = 34;
        public static final int BR_SMAX = 35;
    }

    

    // Intake
    public static final int MAGAZINE_PCM_CAN_ID = CAN.PCM2;
    public static final int MAGAZINE_UP_PCM = 0;
    public static final int MAGAZINE_DOWN_PCM = 1;
    public static final int MAGAZINE_PWM = 9;

    public static final int INTAKE_SPARK_PWM = 0;
    public static final int UPPER_SHOOTER_TALON_CAN = 18;
    public static final int LOWER_SHOOTER_TALON_CAN = 19;

    public static final int INTAKE_PCM_CAN_ID = CAN.PCM1;
    public static final int INTAKE_UP_SOLENOID_PCM = 4;
    public static final int INTAKE_DOWN_SOLENOID_PCM = 5;

   

    // Gearshifter
    public static final int GEARSHIFT_PCM_CAN_ID = CAN.PCM1;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;

    // Climber
    public static final int CLIMBER_PCM_CAN_ID = CAN.PCM2;
    public static final int ARMSOLENOID_LOW_CANID = 0;
    public static final int ARMSOLENOID_HIGH_CANID = 1;
    public static final int WN_SPARKMAX_CANID = 16; // Winch Motor
    public static final int CLIMB_ARM_TALON_CANID = 18; // rotate arm

    // Control Panel Manipulator
    public static final int PANEL_LIMIT_SWITCH_CH = 0;
    public static final int PANEL_ROTATION_CANID = 12;// placeholderA
    public static final int PCM_ID = 1;
    public static final int PANEL_PISTON_FORWARD_PCM = 2;
    public static final int PANEL_PISTON_REVERSE_PCM = 3;

    // Auto Delays - values to be adjusted later with testing
    public static final double DELAY_A = 0.0;
    public static final double DELAY_B = 3.0;
    public static final double DELAY_C = 6.0;

    // angle based on starting position
    public static final double ANGLE_A = 0;
    public static final double ANGLE_B = 2;
    public static final double ANGLE_C = -2.3;

    // departure angle based on starting position
    public static final double LIMELIGHT_DEPARTURE_ANGLE_A = 10;
    public static final double LIMELIGHT_DEPARTURE_ANGLE_B = 2;
    public static final double LIMELIGHT_DEPARTURE_ANGLE_C = -2.3;

    // departure angle based on starting position
    public static final double LIDAR_DEPARTURE_ANGLE_A = -17.0;
    public static final double LIDAR_DEPARTURE_ANGLE_B = 0.0;
    public static final double LIDAR_DEPARTURE_ANGLE_C = 21.0;

    // Limelight Area based on starting position
    public static final double AREA_A = 2.5;
    public static final double AREA_B = 2.9;
    public static final double AREA_C = 2.6;

    // Limelight Area of Departure based on starting possition
    public static final double DEPARTURE_AREA_A = 2.7;
    public static final double DEPARTURE_AREA_B = 2.7;
    public static final double DEPARTURE_AREA_C = 2.7;

    // camera paths
    public static final String FRONT_DRIVE_CAMERA_PATH = "/dev/video1";
    public static final String REAR_DRIVE_CAMERA_PATH = "/dev/video0";
    public static final String ARM_CAMERA_PATH = "/dev/video2";

    public static final int FLOOR_SENSOR = 17; // Time of Flight sensor, measures distance from floor

    public static final class RobotPhysical {
        public static final double BUMPER_TO_LIDAR = 100; // mm 
        public static final double LIDAR_TO_LIDAR = 348;  // mm 

        //useful if we do modeling for tracking
        public static final double Mass = 145;  // lbs with battery and code loaded

        //chassis  
        public static final double WheelDiameter = 7.5; // inches
        public static final double WheelAxelDistance = 25.5; // inches
        
    }

    /**
     * Subsystem constants
     * 
     * Maybe be used directly by the subsystem or passed as args in construction 
     * depending on the need.
     * 
     *    <subsys>.data  convention 
     */
    public static final class LIDAR {
        public static final double SAMPLE_mS = 20; // in ms
        //unused public static final double COLOR_SAMPLE_mS = 100; // in ms
        //unused public static final int LOG_REFRESH_RATE_mS = 100; // in ms
    }

    public static final class DriverPrefs {
        public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
        public static final double RotationExpo = 0.9;   // non-dim [0.0 -1.0]
        public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
    }

    public static final class DriveTrain {
        // motor constraints
        public static final double motorMaxRPM= 5600; // motor limits
        public static final double maxFPS = 14;          // max speed in feet/sec
        public static final double maxRotDPS = 100;     // max rotation rate in deg/sec
        
        // PIDS are in the SparkMax, PIDFControler is used to hold the values
        // for initializing the hardware. The PID object not run on RIO.
        public static final PIDFController pidValues =
             new PIDFController(0.00005, 0.0, 0.0, 0.00025); // P, I, D, FF

        // shifter settings
        public static final int shiftCount = 5;       // frames to wait on vel measurement
        public static final double vShiftLow = 1.5;   // ft/sec shift to low
        public static final double vShiftHigh = 6.8;  // ft/sec shift to high

        // Other constraints
        public static final int smartCurrentMax = 60; //amps in SparkMax
        public static final int smartCurrentLimit = 35; //amps in SparkMax

    }

    public static final class Shooter {

    }

    public static final class Intake {

    }

    public static final class Climber {

    }

    /**
     * Command Constants - used to construct various commands.
     * 
     */
    public static final class ShooterOnCmd {
        public static ShooterOn.Data data = new ShooterOn.Data();
        static {
            data.BackupSec = .1;    // seconds
            data.Tolerance = .025;  //2.5%
            data.HighGoal = new FlywheelRPM(1900, 1900);
            data.LowGoal =  new FlywheelRPM(1000, 1000);
        }
       
    }


}
