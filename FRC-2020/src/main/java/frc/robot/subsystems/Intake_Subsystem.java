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
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PWM;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;
import frc.robot.util.misc.PIDFController;

public class Intake_Subsystem extends MonitoredSubsystemBase implements Logger {
  public static final double USE_CURRENT_ANGLE = 0.0;
  public static final double SAFE_INTAKE_ANGLE = 28.0;   //Mag angle for raising Intake
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
  private NetworkTableEntry nt_autoShooterMode;
  private NetworkTableEntry nt_galacticPath;

  // Intake
  final Spark intake_spark = new Spark(PWM.INTAKE);
  final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_PCM_CAN_ID, INTAKE_UP_SOLENOID_PCM, INTAKE_DOWN_SOLENOID_PCM);

  // Magazine is used for a few controls 
  final Magazine_Subsystem magazine;
  final MagazinePositioner positioner;

  // Flywheels 
  final FlyWheel  upper_shooter; 
  final FlyWheel  lower_shooter; 

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
   * RPMSet and Data are static classes to help configure flywheel speeds
   */
    static class FlywheelRPM {
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
    public FlywheelRPM copy(FlywheelRPM src) {
      this.upper = src.upper;
      this.lower = src.lower;
      return this;
    }

    public FlywheelRPM set( double upper, double lower) {
      this.upper = upper;
      this.lower= lower;
      return this;
    }

    public FlywheelRPM minus(FlywheelRPM a, FlywheelRPM b) {
      this.upper = a.upper - b.upper;
      this.lower = a.lower - b.lower;
      return this;
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
    public double velTol; // percent velocity Shooter must be at or below to fire

    public ShooterSettings(double vel, double rps, double angle, double velTol) {
      this.vel = vel; 
      this.rps = rps;
      this.angle = angle;
      this.velTol = velTol;
    }
    public ShooterSettings(ShooterSettings s) {this(s.vel, s.rps, s.angle, s.velTol);}
    public ShooterSettings() {this(0.0, 0.0, 0.0, 0.1);}
  }

  // default ShooterSetting that let you shoot 
  final ShooterSettings defaultShooterSettings = new ShooterSettings(38.0, 10.0, USE_CURRENT_ANGLE, 0.01);

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
  private boolean m_readyToShoot = false;  // looks at setpoint
  ShooterSettings m_setpoint;              // current shooter setpoint, angle, flywheel speeds
  private boolean autoShootingModeOn = false;
  

  /**
   * 
   */
  public Intake_Subsystem() {
    // Construct the magazine, keep references to it and positioner
    magazine = new Magazine_Subsystem();
    positioner = magazine.getMagPositioner();

    SendableRegistry.setName(this,"Intake", "shooter");
    upper_shooter = new FlyWheel(CAN.SHOOTER_UPPER_TALON, Shooter.upperFWConfig);
    lower_shooter = new FlyWheel(CAN.SHOOTER_LOWER_TALON, Shooter.lowerFWConfig);

    table = NetworkTableInstance.getDefault().getTable("Shooter");
    nt_upperRPM = table.getEntry("UpperRPM/value");
    nt_lowerRPM = table.getEntry("LowerRPM/value");
    nt_autoShooterMode = table.getEntry("ShootingAutoMode");
    nt_galacticPath = table.getEntry("GalacticPathIsRed");
    table.getEntry("GalacticPathIsRed").setPersistent();

    // build out matrix to calculate FW RPM from [omega , Vel] for power cell
    VelToRPM.set(0, 0, Shooter.PCEffectiveRadius / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(0, 1,  1.0 / Shooter.lowerFWConfig.flywheelRadius);
    VelToRPM.set(1, 0, -Shooter.PCEffectiveRadius / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.set(1, 1, 1.0 / Shooter.upperFWConfig.flywheelRadius);
    VelToRPM.times(0.5);  // common factor 1/2

    m_setpoint = defaultShooterSettings;
  }

  @Override
  public void monitored_periodic() {
    //measure flywheel rpm and calculate our error 
    actual.set(upper_shooter.getRPM(), lower_shooter.getRPM());
    error.minus(target, actual);

    // monitor if the  shooter rpm and angle is ready to shoot
    isAtGoal();
  }

  public void setGalacticPathIsRed(){
    nt_galacticPath.setBoolean(true);
  }

  public void setGalacticPathIsBlue(){
    nt_galacticPath.setBoolean(false);
  }

  public boolean getGalacticPathIsRed(){
    return nt_galacticPath.getBoolean(false);
  }

  public void raiseIntake() {
    // can't have magazine up with intake up, force it down.
    // Command Layer needs to confirm this.
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

  public boolean isIntakeOn() {
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
   FlywheelRPM calculateGoals(ShooterSettings s) {
    final double radPerSec2revPerMin = 60 / (2.0*Math.PI);
    // order of input vector must match VelToRPM matrix order
    vel.set(0, 0, s.rps*2.0*Math.PI); //rad/s
    vel.set(1, 0, s.vel);             //ft/s  
    var omega = VelToRPM.times(vel).times(radPerSec2revPerMin);
    return new FlywheelRPM(omega.get(0,0), omega.get(1,0));
  }


  public void setShooterSettings(ShooterSettings s) {
    // default setting will shoot from any angle
    s = (s == null) ? defaultShooterSettings : s;
    m_setpoint = s;
  }
  
  public ShooterSettings getShooterSettings() { 
    return m_setpoint;
  }

  /**
   * spinupShooter()
   * 
   * Preferred interface for controlling Flywheels
   * This will not shoot, that requires control of the belt and
   * perhaps some targing feedback.
   * 
   * Uses the m_setpoint settings
   * 
   * Use this interface to Warm-up the flywheels.
   * 
   */
  public void spinupShooter() {
    // shooter will run at the target goals
    shooterOn(calculateGoals(m_setpoint));
  }

  /**
   * Commands shooter flwwheels to target RPM
   * 
   * 
   * @param goals   (flywheel rpm goals)
   */
  void shooterOn(FlywheelRPM goals) {
    shooterIsOn = true;
    // save the targets for at goal calcs
    target.copy(goals);
    
    upper_shooter.setRPM(target.upper);
    lower_shooter.setRPM(target.lower);
  }

  /**
   * Shouldn't use this api, debugging only.
   */
  void shooterOnPercent(double upperPct, double lowerPct) {
    shooterIsOn = true;
      upper_shooter.setPercent(upperPct); 
      lower_shooter.setPercent(lowerPct); 
  }

  public boolean isShooterOn() {
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
   * isAtGoal(ShooterSettings goal)
   * @param goal where we want the shooter, speed & angle
   * @return true - ready to shoot
   *         false - either angle or flywheels not up to speed
   */

  boolean isAtGoal() {
    m_readyToShoot = false;
    
    // see if we are at the given goal, keep state var updated
    boolean fw_ready = isAtGoalRPM(m_setpoint.velTol);
    boolean angle_ready = (m_setpoint.angle == USE_CURRENT_ANGLE) ? true : positioner.isAtSetpoint(m_setpoint.angle);
    m_readyToShoot = fw_ready && angle_ready;
    return m_readyToShoot;
  }

  /**
   * isReadyToShoot - public API, calculated in periodic from m_setpoint
   * @return  true  angle/speed ready
   *          false something is not ready
   */
  public boolean isReadyToShoot() {return m_readyToShoot;}

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
  boolean isAtGoalRPM(double tol) {
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

  public void toggleAutoShootingMode() {
    autoShootingModeOn = autoShootingModeOn ? false : true;
  }

  public boolean getAutoShootingMode(){
    return autoShootingModeOn;
  }

  @Override
  public void log() {
    // Put any useful log message here, called about 10x per second
    nt_lowerRPM.setDouble(actual.upper);
    nt_upperRPM.setDouble(actual.lower);
    nt_autoShooterMode.setBoolean(autoShootingModeOn);  
  }

/**
 * 
 *  addDashboardWidgest
 * 
 * 
 * @param layout  - panel to put the data on
 */
  public void addDashboardWidgets(ShuffleboardLayout layout) {
    /*layout.addNumber("MO/Upper", upper_shooter::getMotorOutputPercent ).withSize(2, 1) ;
    layout.addNumber("MO/Lower", lower_shooter::getMotorOutputPercent ) ;
    layout.addNumber("RPM/Upper", () -> actual.upper).withSize(2,1);
    layout.addNumber("RPM/Lower", ()-> actual.lower);
    layout.addNumber("RPM/ErrorL", () -> error.lower );
    */
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
