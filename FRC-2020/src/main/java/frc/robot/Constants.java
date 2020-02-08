/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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

    public static final double DT = 0.02; // 20ms framerate 50Hz
    public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

    public static final int FRONT_LEFT_LIDAR = 21;
    public static final int FRONT_RIGHT_LIDAR = 22;
    public static final double LIDAR_SAMPLE_TIME = 100; // in ms
    public static final double COLOR_SAMPLE_TIME = 100; // in ms
    public static final int LOG_REFRESH_RATE = 100; // in ms

    // Intake
    public static final int INTAKE_UP_DIO = 4;
    public static final int INTAKE_DOWN_DIO = 4;
    public static final int INTAKE_TALON_CAN = 20;
    public static final int MAGAZINE_TALON_CAN = 21;
    public static final int UPPER_SHOOTER_TALON_CAN = 22;
    public static final int LOWER_SHOOTER_TALON_CAN = 23;
    public static final int ELEVATOR_TALON_CAN = 24;
    public static final int ELEVATOR_PCM_ID = 1;
    public static final int ELEVATOR_UP_SOLENOID_PCM = 1;
    public static final int ELEVATOR_DOWN_SOLENOID_PCM = 2;

    public static final int FL_SPARKMAX_CANID = 11;
    public static final int ML_SPARKMAX_CANID = 12;
    public static final int BL_SPARKMAX_CANID = 13;
    public static final int FR_SPARKMAX_CANID = 24;
    public static final int MR_SPARKMAX_CANID = 22;
    public static final int BR_SPARKMAX_CANID = 23;

    public static final int GEARSHIFT_PCM_ID = 1;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
