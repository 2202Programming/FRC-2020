/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.intake.ShooterOn;
import frc.robot.subsystems.Intake_Subsystem.FlywheelRPM;

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

        // Other CAN ID - Subsystem_location

    }

    // Lidar
    public static final int FRONT_LEFT_LIDAR_CANID = 22;
    public static final int FRONT_RIGHT_LIDAR_CANID = 21;
    public static final double LIDAR_SAMPLE_mS = 20; // in ms
    public static final double COLOR_SAMPLE_mS = 100; // in ms
    public static final int LOG_REFRESH_RATE_mS = 100; // in ms

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

    // Drivetrain
    public static final int FL_SPARKMAX_CANID = 30;
    public static final int ML_SPARKMAX_CANID = 31;
    public static final int BL_SPARKMAX_CANID = 32;
    public static final int FR_SPARKMAX_CANID = 33;
    public static final int MR_SPARKMAX_CANID = 34;
    public static final int BR_SPARKMAX_CANID = 35;

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

    /**
     * Subsystem constants
     */
    public static final class LIDAR {

    }

    public static final class DriveTrain {

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
