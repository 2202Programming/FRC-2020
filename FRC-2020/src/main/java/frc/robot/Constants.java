/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.DrivePreferences;
import frc.robot.subsystems.Intake_Subsystem.FlyWheelConfig;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.util.misc.PIDFController;

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

        // Climber Talon  
        public static final int CLIMBER_TALON = 14; 

        //Shooter
        public static final int SHOOTER_UPPER_TALON = 18; 
        public static final int SHOOTER_LOWER_TALON = 19;
        
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

        public static final int MAG_SMAX = 55;
    }

    // PWM assignments on the Rio
    public static final class PWM {
      public static final int INTAKE = 0;
      public static final int MAGAZINE = 9; 
    }
    
    // Digital IO on the RIO
    public static final class DigitalIO {
      public static final int LEFT_CHASSIS_ENCODER_A = 0;
      public static final int LEFT_CHASSIS_ENCODER_B = 1;
      public static final int MAGAZINE_GATE = 2;  
      public static final int MAGAZINE_GATE_PWR = 4;  
      public static final int RIGHT_CHASSIS_ENCODER_A = 5;
      public static final int RIGHT_CHASSIS_ENCODER_B = 6;
    }


    // Analog IO on the RIO
    public static final class AnalogIn {
      public static final int MAGAZINE_ANGLE = 0;
    }

    //Pnumatics control 1 
    public static final class PCM1 {
        public static final int CLIMBER_EXTEND = 3;
        public static final int CLIMBER_RETRACT = 2;

      }

    //Pnumatics control 2 
    public static final class PCM2 {
      public static final int MAG_LOCK = 0;
      public static final int MAG_UNLOCK = 1;
    }

    // Intake
    public static final int INTAKE_PCM_CAN_ID = CAN.PCM1;
    public static final int INTAKE_UP_SOLENOID_PCM = 4;
    public static final int INTAKE_DOWN_SOLENOID_PCM = 5;

    // Gearshifter
    public static final int GEARSHIFT_PCM_CAN_ID = CAN.PCM1;
    public static final int GEARSHIFTUP_SOLENOID_PCM = 0;
    public static final int GEARSHIFTDOWN_SOLENOID_PCM = 1;


    // Control Panel Manipulator (unused)
    public static final int PANEL_LIMIT_SWITCH_CH = 0;
    public static final int PANEL_ROTATION_CANID = 12;// placeholderA
    public static final int PCM_ID = 1;
    public static final int PANEL_PISTON_FORWARD_PCM = 2;
    public static final int PANEL_PISTON_REVERSE_PCM = 3;

    // Auto Delays - values to be adjusted later with testing
    public static final double DELAY_A = 1.0; //sideboard button 1 on
    public static final double DELAY_B = 2.0; //sideboard button 2 on
    public static final double DELAY_C = 4.0; //sideboard button 1 and 2 on

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
        public static final double WheelDiameter = 6.0; // inches, nominal
        public static final double WheelAxleDistance = 2.22418; //(robot char 3/20/21)    //25.5/12.0; // feet 25.5 mid-wheel, 26.75 outer wheel

        //wheel wear compensation - adjust when distance is off by small amount
        public static final double WheelWearLeft = 0.99;   //[percent] of nominal
        public static final double WheelWearRight = 0.99;  //[percent] of nominal
        
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
        public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
        public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
    }

    public static final class DriveTrain {
        // motor constraints
        public static final double motorMaxRPM= 5600;    // motor limit
        
        // common Kff - based on measured open loop response 
        public static final double Kff = 0.00016666;
      
        // DL/Dustin - kD=100 was good compromise for damping.  200 deg/sec rot was good too
        public static final DrivePreferences driverPreferences = new DrivePreferences();
        static {
          driverPreferences.pidSlot = 0;
          driverPreferences.rpmPID =  new PIDFController(0.000155, 0.0, 100.0, Kff);
          driverPreferences.slewRateLimit = 0.6;   //[s]  time to max output
          driverPreferences.maxRotRate = 150.0;    //[deg/s]
          driverPreferences.maxVelocity = 7.5;     //[ft/s]
        }

        //20 improved turn oscillation; 80 starting to feel slow
        //Kp 0.000075 maybe less turn oscillations
        //JR 3/17 settings 0.000155, 0.0, 0.0, 0.00016666
        public static final DrivePreferences trackerPreferences = new DrivePreferences();
        static {
          trackerPreferences.pidSlot = 1;
          trackerPreferences.rpmPID =  new PIDFController(0.000155, 0.0, 200.0, Kff);
          trackerPreferences.slewRateLimit = 0.1;   //[s]  time to max output
          trackerPreferences.maxRotRate =  130.0;    //[deg/s]
          trackerPreferences.maxVelocity = 7.0;     //[ft/s]
        }

        // shifter settings
        public static final int shiftCount = 150;       // frames to wait on vel measurement
        public static final double vShiftLow = 1.5;   // ft/sec shift to low
        public static final double vShiftHigh = 99; // no auto to high 6.8;  // ft/sec shift to high

        // Other constraints
        public static final int smartCurrentMax = 60;  //amps in SparkMax, max setting
        public static final int smartCurrentLimit = 35; //amps in SparkMax, inital setting

        // Acceleration limits
        public static final double slewRateMax = 2;      //sec limits adjusting slewrate 

        public static final boolean safetyEnabled = false; 
    }

    public static final class Shooter {
      // Power Cell info
      public static final double PowerCellMass = 3.0 / 16.0; // lbs
      public static final double PCNominalRadius = 7.0 / 2.0 / 12.0; // feet - power cell
      public static final double PCEffectiveRadius = 4.75 / 2.0 / 12.0; // feet - compressed radius

      /**
       * Convert Target RPM to [motor-units/100ms] 4096 Units/Rev * Target RPM * 600 =
       * velocity setpoint is in units/100ms
       */
      public static final double kRPM2Counts = 4096.0/600.0; // MU-100 (no gearing)
      public static final double kMaxMO = 1023;  // max Motor output

      // Flywheel info
      // Flywheel maxOpenLoopRPM and gear ratio are used to calculate kFF in shooter
      public static FlyWheelConfig upperFWConfig = new FlyWheelConfig();
      static {
        upperFWConfig.maxOpenLoopRPM = 3074;  // estimated from 2000 RPM test
        upperFWConfig.gearRatio = 5.0;        // upper is 5:1 (motor:fw)
        upperFWConfig.sensorPhase = false;
        upperFWConfig.inverted = false;
        upperFWConfig.flywheelRadius = 2.0 / 12.0; // feet
        upperFWConfig.pid = new PIDFController(0.08, 0.00015, 4.0, 0); // kP kI kD kFF
        upperFWConfig.pid.setIzone(1800);
      }

      public static FlyWheelConfig lowerFWConfig = new FlyWheelConfig();
      static {
        lowerFWConfig.maxOpenLoopRPM = 5100;
        lowerFWConfig.gearRatio = 3.0;         // lower fw gear 3:1  (motor:flywheel)
        lowerFWConfig.sensorPhase = false;
        lowerFWConfig.inverted = true; 
        lowerFWConfig.flywheelRadius = 1.25 / 12.0;   //feet 
        lowerFWConfig.pid = new PIDFController(0.08, 0.00015, 4.0, 0);   // kP kI kD kF 
        lowerFWConfig.pid.setIzone(1800);
      }

    }

    public static final class Intake {
      //angle MAGAZINE must be below to raise intake
      public static final double MAG_UP_ANGLE = 28.0; // degrees
    }

    public static final class Climber {

    }

    /**
     * Command Constants - used to construct various commands.
     * 
     */
    public static final class ShooterOnCmd {
        public static final double VEL_TOL = 0.1;  
        public static ShooterSettings dataHigh = new ShooterSettings(37.0, 0, 40.8, VEL_TOL);  //vel, rps, angle, tol
        public static ShooterSettings dataLow  = new ShooterSettings(37.0, 0, 35.0, VEL_TOL);  //vel, rps, angle
    }

    // IA ShooterSettings
    public static final class InterstellarSettings {
      public static final double VEL_TOL = 0.05; 

      // 2d target
     // public static ShooterSettings ssZone1 = new ShooterSettings(32.0, 5, 53.5, VEL_TOL);  //vel, rps, angle, tol
     // public static ShooterSettings ssZone2 = new ShooterSettings(36.0, 5, 42.5, VEL_TOL);  //vel, rps, angle, tol
     // public static ShooterSettings ssZone3 = new ShooterSettings(38.0, 5, 38.5, VEL_TOL);  //vel, rps, angle, tol
     // public static ShooterSettings ssZone4 = new ShooterSettings(40.0, 5, 38//, VEL_TOL);  //vel, rps, angle, tol
      
      // 3d target
       public static ShooterSettings ssZone1 = new ShooterSettings(32.0, 5, 48.7, VEL_TOL);  //vel, rps, angle, tol
       public static ShooterSettings ssZone2 = new ShooterSettings(36.0, 5, 40.5, VEL_TOL);  //vel, rps, angle, tol
       public static ShooterSettings ssZone3 = new ShooterSettings(38.0, 5, 38.0, VEL_TOL);  //vel, rps, angle, tol
       public static ShooterSettings ssZone4 = new ShooterSettings(43.0, 5, 37.5, VEL_TOL);  //vel, rps, angle, tol
    }
    
    public static final class RamseteProfile {
      // These characterization values MUST be determined either experimentally or
      // theoretically for *your* robot's drive.
      // The Robot Characterization Toolsuite provides a convenient tool for obtaining
      // these values for your robot.
      //3.20.21 updated robo char 
      public static final double ksVolts = 0.203;   //0.123; 
      public static final double kvVoltSecondsPerFoot= 1.49;   //1.48; 
      public static final double kaVoltSecondsSquaredPerFoot = 0.1; // 0.15; 

      public static final double kPDriveVel =2.05;  /// 2.44; 

      // Change these to go faster/slower for commands using ramsete and VoltageDrive

      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds - DPL not sure about how to convert to feet.
      public static final double kRamseteB = .18;
      public static final double kRamseteZeta = 0.9;
      //public static final double kTrackwidthFeet = 2.135; //2.08333;   // from char=1.4348; //updated2

      // Use the kinematics from the drivetrain, computed from RobotPhysical constants above
      // public static final DifferentialDriveKinematics kDriveKinematics =
      // new DifferentialDriveKinematics(kTrackwidthFeet);

      
    public static final double kMaxSpeedFeetPerSecond = 5;
    public static final double kMaxAccelerationFeetPerSecondSquared = 1;
      
    }

}
