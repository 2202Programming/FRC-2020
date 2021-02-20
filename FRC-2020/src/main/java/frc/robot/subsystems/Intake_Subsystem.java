/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import static frc.robot.Constants.INTAKE_DOWN_SOLENOID_PCM;
import static frc.robot.Constants.INTAKE_PCM_CAN_ID;
import static frc.robot.Constants.INTAKE_UP_SOLENOID_PCM;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Intake;
import frc.robot.Constants.PWM;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.util.misc.PIDFController;

public class Intake_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Intake_Subsystem.
   * 
   * Use the WPI_TalonSRX wrapper around the lower level TalonSrx because it
   * implements the WPI SpeedController, Sendable.
   * 
   * 
   * Who When What DPL 2/09/2020 removed publics, use WPI_TalonSRX, Gear gain Also
   * removed some debug code. DPL 12/12/2020 setup for Kevin' PID UX, Moved motor
   * stuff to flywheel class invert flag for feedback, setInvert for motor
   * polarity.
   * 
   * DPL 12/15/2020 tested in lab, sort of worked but oscilated. Found Ki was 10x
   * what we tested 12/12 in DifferentialShooter branch. Changed, need to retest.
   *
   * DPL 1/30/2021 Tuned for new wheel layout. Added Izone in small region to zero
   * error Kd = at 80% of value that caused oscilation, want as large as possible
   * to resist speed change when ball is being launched. Kff = based on open loop
   * speed at max RPM
   * 
   * DPL 2/1/2021 moved constants to Constants.java split for different
   * upper/lower flywheel diameter added velocty and rotation as main control
   * inputs
   * 
   */

  private NetworkTable table;
  private NetworkTableEntry nt_upperRPM;
  private NetworkTableEntry nt_lowerRPM;

  // Intake
  Spark intake_spark = new Spark(PWM.INTAKE);
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_PCM_CAN_ID, INTAKE_UP_SOLENOID_PCM, INTAKE_DOWN_SOLENOID_PCM);
 
  // Magazine is used for a few controls 
  Magazine_Subsystem magazine;

  // Flywheels 
  FlyWheel  upper_shooter; 
  FlyWheel  lower_shooter; 

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
   * Velocity units at 100% output.
   * 
   *  Calculate Kf to get us close to desired speed and pid will fine tune it.
   * 
   *     [FW-RPM] = flywheel rpm
   *     [MU-100] = motor units per 100mS which is the controller's vel uint, [MU] for short
   *     2000 RPM * 34.133 [MU-100ms]/[FW-RPM]  = 68267 MU/FW-RPM
   * 
   *     New Flywheels - need new KF. Compromise between 1000 and 2000 Open-loop
   *     Kf = .00934 as measured/calculated 12/19/2020  DPL/Alek O.
   *                       
   *   2/6/21  Kff now calcualted from max FW RPM 
   */

  public static class FlyWheelConfig {
    public PIDFController pid;
    public double maxOpenLoopRPM; 
    public double gearRatio;           // account for gearbox reduction to flywheel
    public boolean sensorPhase;
    public boolean inverted;
    public double flywheelRadius;
    public double FWrpe2MU;            // FlywheelRPM to Motor-Units  (includes gearing)
  };

  /**
   * RPMSet and Data are static classes to help configure the comma
   */
    public static class FlywheelRPM {
    public double upper;
    public double lower;

    public FlywheelRPM() { this(0.0, 0.0); }
    public FlywheelRPM(FlywheelRPM c) { this(c.lower, c.upper); }
    public FlywheelRPM(double lower, double upper) {
      this.upper = upper;
      this.lower= lower;
    }

    public double getAverage() { 
      return (0.5*(upper + lower));
    }
    /**
     * Copy(src) - copy data into structure from a source
     * @param src
     */
    public void copy(FlywheelRPM src) {
      this.upper = src.upper;
      this.lower = src.lower;
    }
    public String toString() {
      return Double.toString(upper) + "/" + Double.toString(lower);
    }
  }

  /**
   * ShooterSettings - simple struture to group the shooters settings.
   * 
   *   Rotations/sec is used as input, remember to convert to RADIAN/SEC
   *   to calculate speeds
   */
  public static class ShooterSettings 
  {
    public double vel;   // power cell ft/sec 
    public double rps;   // power cell rotations/sec 
    public double angle; // angle to set the shooter output

    public ShooterSettings(double vel, double rps, double angle) {
      this.vel = vel; 
      this.rps = rps;
      this.angle = angle;
    }
    public ShooterSettings(ShooterSettings s) {this(s.vel, s.rps, s.angle);}
    public ShooterSettings() {this(0.0, 0.0, 0.0);}
  }

  // All RPM are in FW-RPM, not motor.
  FlywheelRPM actual = new FlywheelRPM();
  FlywheelRPM target = new FlywheelRPM();
  FlywheelRPM error = new FlywheelRPM();

  //Transfrom from [ w, V] [W_lower, W_upper]
  final Matrix<N2,N2> VelToRPM = new Matrix<>(Nat.N2(), Nat.N2() );
  Vector<N2> vel = new Vector<N2>(Nat.N2());
  
  //state variables
  private boolean intakeIsOn = false;
  private boolean shooterIsOn = false;

  public Intake_Subsystem() {
    // Construct the magazine 
    magazine = new Magazine_Subsystem(this);

    SendableRegistry.setName(this,"Intake", "shooter");
    upper_shooter = new FlyWheel(CAN.SHOOTER_UPPER_TALON, Shooter.upperFWConfig);
    lower_shooter = new FlyWheel(CAN.SHOOTER_LOWER_TALON, Shooter.lowerFWConfig);

    table = NetworkTableInstance.getDefault().getTable("Shooter");
    nt_upperRPM = table.getEntry("UpperRPM");
    nt_lowerRPM = table.getEntry("LowerRPM");

    // build out matrix to calculate FW RPM from [omega , Vel] for power cell
    VelToRPM.set(0, 0, Shooter.PCEffectiveRadius / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(0, 1,  1.0 / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(1, 0, -Shooter.PCEffectiveRadius / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.set(1, 1, 1.0 / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.times(0.5);  // common factor 1/2

    //TODO: fix magazine.lower(); // must start in down positon
   ///dpl shop debug raiseIntake();    // must start in the up position
   lowerIntake(); //dpl shop debug
  }

  @Override
  public void periodic() {
    // update RPM variables here, because we do controls on them and don't
    // want to have measurement lag.
    actual.upper = upper_shooter.getRPM(); 
    actual.lower = lower_shooter.getRPM();

    error.upper = target.upper - actual.upper;
    error.lower = target.lower - actual.lower;
  }

  public void raiseIntake() {
    // can't have magazine up with intake up, force it down.
    // Magazine goes down faster, so should be ok...
    if (magazine.positioner.get() > Intake.MAG_UP_ANGLE) { 
      magazine.lower();
    }
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerIntake() {
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isIntakeUp() {
    return (intakeSolenoid.get() == Value.kForward);
  }

  public void intakeOn(double motorStrength) {
    intakeIsOn = true;
    intake_spark.set(motorStrength);
  }

  public boolean intakeIsOn() {
    return intakeIsOn;
  }

  public void intakeOff() {
    intakeIsOn = false;
    intake_spark.set(0);
  }

  public Magazine_Subsystem getMagazine() {
    return this.magazine;
  }

  /**
   * Set the Shooter RPM goals from power cell velocity and rotation rate.  
   * 
   * Uses a transformation matrix
   * 
   *   [ VelToRPM<2,2>] * [w, vel ]^T =  [RPM_lower,  RPM_upper]^T
   *   
   *    @param ShooterSettings  
   *       @param pc_omeg_rps   Power Cell w = rotations / sec 
   *       @param pc_fps        Power Cell Vel = ft / sec
   * 
   *
   * @return FlywheelRPM goal
   */
  public FlywheelRPM calculateGoals(ShooterSettings s) {
    final double radPerSec2revPerMin = 60 / (2.0*Math.PI);
    // order of input vector must match VelToRPM matrix order
    vel.set(0, 0, s.rps*2.0*Math.PI); //rad/s
    vel.set(1, 0, s.vel);             //ft/s  
    var omega = VelToRPM.times(vel).times(radPerSec2revPerMin);
    return new FlywheelRPM(omega.get(0,0), omega.get(1,0));
  }

  /**
   * Commands shooter flwwheels to target RPM
   * 
   * @param goals   (flywheel rpm goals)
   */
  public void shooterOn(FlywheelRPM goals) {
    shooterIsOn = true;
    // save the targets for at goal calcs
    target.copy(goals);
    
    upper_shooter.setRPM(target.upper);
    lower_shooter.setRPM(target.lower);
  }

  public void shooterOnPercent(double upperPct, double lowerPct) {
    shooterIsOn = true;
      upper_shooter.setPercent(upperPct); 
      lower_shooter.setPercent(lowerPct); 
  }

  public boolean shooterIsOn() {
    return shooterIsOn;
  }

  public void shooterOff() {
    shooterIsOn = false;
    target.lower = 0;
    target.upper = 0;
    upper_shooter.setPercent(0.0);
    lower_shooter.setPercent(0.0);
  }

  

  /**
   * getShooterAvgRPM()  - average of upper and lower wheels
   * 
   * Warning, not really meaningful if flywheels have different diameters.
   * 
   * @return
   */
  public double getShooterAvgRPM() {    
    return actual.getAverage();
  }

  /**
   * getUpperRPM(), getLowerRPM()
   *   gets value measures in periodic() call
   * 
   * @return
   */
  public double getUpperRPM() {return actual.upper;}
  public double getLowerRPM() {return actual.lower;}
  public void getFlywheelRPM(FlywheelRPM ref) {
    ref.copy(actual);  //puts actual values into ref
  }

  /**
   * getUpperTargetRPM(), getLowerTargetRPM()
   *  
   * @return last commanded upper/lower Flywheel target RPM
   */
  public double getUpperTargetRPM() {return target.upper;}
  public double getLowerTargetRPM() {return target.lower;}
  public void getFlywheelTargetRPM(FlywheelRPM ref) {
    ref.copy(target);  //puts values into given ref
  }

  /**
   * Checks to see if both flywheels are at the desired speed
   * All goals are given in Flywheel RPM.
   * 
   * Both flywheels use positive commands.
   * 
   * @param upperGoal
   * @param lowerGoal
   * @param tol
   * @retur
   */
  public boolean atGoalRPM(double tol) {
    // return true if BOTH upper and lower are within tolerance 
    return ((Math.abs(error.upper) < (target.upper*tol)) &&
            (Math.abs(error.lower) < (target.lower*tol)) );
  }

  public double getShooterPercent() {
    // Get the current output percent of the upper and lower shooter motors
    double upperPerc = upper_shooter.getMotorOutputPercent();
    double lowerPerc = lower_shooter.getMotorOutputPercent();
    // Gets the lower of the upper and lower shooter current speed
    return (upperPerc + lowerPerc)*0.5;
  }

  @Override
  public void log() {
    // Put any useful log message here, called about 10x per second
    nt_lowerRPM.setDouble(actual.upper);
    nt_upperRPM.setDouble(actual.lower);
  }

/**
 * 
 *  addDashboardWidgest
 * 
 * 
 * @param layout  - panel to put the data on
 */
  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.addNumber("MO/Upper", upper_shooter::getMotorOutputPercent ).withSize(2, 1) ;
    layout.addNumber("MO/Lower", lower_shooter::getMotorOutputPercent ) ;
    layout.addNumber("RPM/Upper", () -> actual.upper).withSize(2,1);
    layout.addNumber("RPM/Lower", ()-> actual.lower);
    layout.addNumber("RPM/ErrorL", () -> error.lower );
  }


/**
 *  Flywheel handles motor and gearing for the shooter flywheels.
 * 
 */
  public class FlyWheel {
    //Talon Slot stuff, we just use slot 0
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    TalonSRXConfiguration srxconfig;
    WPI_TalonSRX motor;     //this could be a generic motor controller...
    
    final double FWrpm2Counts;    // flywheel RPM given motor-unit counts (f(gear, meas-period))
    final double MUCounts2FWrpm;  // motor units (counts/100ms) to FW RPM (1/FWrpm2Counts)

    FlyWheel(int CAN_ID, FlyWheelConfig cfg) {
      srxconfig = new TalonSRXConfiguration();
      motor = new WPI_TalonSRX(CAN_ID);
      motor.setInverted(cfg.inverted);
    
      // flywheel constants RPM given motor-unit counts (f(gear, meas-period))
      FWrpm2Counts = Shooter.kRPM2Counts * cfg.gearRatio;  //motor counts are bigger, motor spins faster than FW    
      MUCounts2FWrpm  = 1.0 / FWrpm2Counts;  // motor units (counts/100ms) to FW RPM 

      // use max rpm and max motor out to calculate kff
      double kff = Shooter.kMaxMO / (cfg.maxOpenLoopRPM * FWrpm2Counts);
      cfg.pid.setF(kff);

      ErrorCode lasterr = motorConfig(cfg);
      if (lasterr.value != 0 ) {
        System.out.println("Flywheel motor error:" + lasterr.value + "  CANID=" + CAN_ID);
      }
    }

    ErrorCode motorConfig(FlyWheelConfig cfg) {
      /* Factory Default all hardware to prevent unexpected behaviour */
      motor.configFactoryDefault();
      
      // use the config to set all values at once
      cfg.pid.copyTo(srxconfig.slot0);

      srxconfig.slot1 = srxconfig.slot0;
      motor.configAllSettings(srxconfig);

      /* Config sensor used for Primary PID [Velocity] */
      motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
      motor.setSensorPhase(cfg.sensorPhase);   // fix feedback direction
      motor.setNeutralMode(NeutralMode.Coast);

      /* Config the peak and nominal outputs */
      motor.configNominalOutputForward(0, kTimeoutMs);
      motor.configNominalOutputReverse(0, kTimeoutMs);
      motor.configPeakOutputForward(1, kTimeoutMs);
      motor.configPeakOutputReverse(-1, kTimeoutMs);
      return motor.getLastError();
    }

    /**
     * Gets RPM as measured at the flywheel 
     * @return flywheel_rpm
     */
    public double getRPM() {
      double vel_mu = motor.getSelectedSensorVelocity();   //motor units
      return vel_mu * MUCounts2FWrpm;   
    }
    public double getMotorOutputPercent() {
      return motor.getMotorOutputPercent();
    }

    public void  setRPM(double fw_rpm) {
      double sp = fw_rpm * FWrpm2Counts;
      motor.set(ControlMode.Velocity, sp);
    }

    public void setPercent(double pct) {
      motor.set(ControlMode.PercentOutput, pct); 
    }
  } //FlyWheel

}
