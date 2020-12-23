/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.*;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.util.misc.PIDFController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Intake_Subsystem.
   * 
   * Use the WPI_TalonSRX wrapper around the lower level TalonSrx because it
   * implements the WPI SpeedController, Sendable.
   * 
   * 
   * Who When What
   *  DPL 2/09/2020 removed publics, use WPI_TalonSRX, Gear gain Also removed some debug code.
   *  DPL 12/12/2020  setup for Kevin' PID UX, Moved motor stuff to flywheel class
   *                  invert flag for feedback, setInvert for motor polarity.
   * 
   *  DPL 12/15/2020  tested in lab, sort of worked but oscilated. Found Ki was 10x what 
   *                  we tested 12/12 in DifferentialShooter branch.  Changed, need to retest.
   */

  // Intake
  Spark intake_spark = new Spark(INTAKE_SPARK_PWM);
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_PCM_CAN_ID, INTAKE_UP_SOLENOID_PCM, INTAKE_DOWN_SOLENOID_PCM);
 
  // magazine
  Spark magazine = new Spark(MAGAZINE_PWM);
  DoubleSolenoid magSolenoid = new DoubleSolenoid(MAGAZINE_PCM_CAN_ID, MAGAZINE_UP_PCM, MAGAZINE_DOWN_PCM);
  
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
   *     MU max out = +/- 1023   - given by CTRE docs
   *     60% motor pct gave  2000 RPM   - measured in the lab 12/12/2020
   *     2000 fw-rpm * Kf  = (0.60)*1023    60% output gave 2000FW-RPM in testing
   * 
   *     Kf = 6138/2000 = 0.3069 [MU/FW_RPM]
  *
   *     [FW-RPM] = flywheel rpm
   *     [MU-100] = motor units per 100mS which is the controller's vel uint, [MU] for short
   *     2000 RPM * 34.133 [MU-100ms]/[FW-RPM]  = 68267 MU/FW-RPM
   * 
   *     New Flywheels - need new KF. Compromise between 1000 and 2000 Open-loop
   *     Kf = .00934 as measured/calculated 12/19/2020  DPL/Alek O.
   * 
   *     Initial Err = 68267 MU
   *     Max Kp contribution = 15% (1023)[MO] = 153 MO
   *      Kp*Error = 153 [MO]
   *      Kp = 153/68267 = 0.002248  [MO / MUerr]  (very safe)
   * 
   *      Kp = 0.1 worked well and still has lots of margin.
   *      KD = 6 damped out with out adding ringing or lag
   *      Ki = 0   not needed for velocity loop
   * 
   * 
   * Use same values as starting point for both upper and lower FW PIDF.                             
   */
  PIDFController pidValues = new PIDFController(0.1, 0.00000, 6, 0.00934);   // kP kI kD kF 

  /**
   * Convert Target RPM to units / 100ms. 4096 Units/Rev * Target RPM * 600 =
   * velocity setpoint is in units/100ms
   * 
   * Gear ratio is 5:1 and sensor is before gearbox 12/12/20
   * (Motor turns 5x the flywheel)
   */

  final double maxOpenLoopRPM = 3330;   // estimated from 2000 RPM test
  final double Gear = 5.0;              // account for gearbox reduction to flywheel
  final double ShooterEncoder = 4096;   // counts per rev motor 
  final double RPM2CountsPer100ms = 600.0; // Vel uses 100mS as counter sample period
  final double kRPM2Counts = (ShooterEncoder) / RPM2CountsPer100ms;  // motor-units (no gearing)

  // All RPM are in FW-RPM 
  double lowerRPM;  //measured in periodic()
  double upperRPM;  //measured in periodic()
  double upperRPM_target;
  double lowerRPM_target;
  
  //state variables
  private boolean intakeIsOn;
  private boolean shooterIsOn;

  public Intake_Subsystem() {
    upper_shooter = new FlyWheel(UPPER_SHOOTER_TALON_CAN, pidValues, false, maxOpenLoopRPM);
    upper_shooter.setMotorTurnsPerFlywheelTurn(Gear);
    lower_shooter = new FlyWheel(LOWER_SHOOTER_TALON_CAN, pidValues, false, maxOpenLoopRPM);
    lower_shooter.setMotorTurnsPerFlywheelTurn(Gear);
    lower_shooter.setInverted(); 

    intakeIsOn = false;
    shooterIsOn = false;

    magazineDown(); // must start in down positon
    raiseIntake();  // must start in the up position
  }

  @Override
  public void periodic() {
    // update RPM variables here, because we do controls on them and don't
    // want to have measurement lag.
    upperRPM = upper_shooter.getRPM(); 
    lowerRPM = lower_shooter.getRPM();

    // This method will be called once per scheduler run
    //lastError = upper_shooter.getLastError();
  }

  public void raiseIntake() {
    // can't have magazine up with intake up, force it down.
    // Magazine goes down faster, so should be ok...
    if (isMagazineUp()) { magazineDown();}
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

  public void magazineOn(double motorStrength) {
    magazine.set(motorStrength);
  }

  public void magazineOff() {
    magazine.set(0);
  }

  /**
   * TCommands shooter flwwheels to target RPM
   * 
   * @param upperRPM_target   (measured at flywheel)
   * @param lowerRPM_target   (measured at flywheel)
   */
  public void shooterOn(double upperRPM_target, double lowerRPM_target) {
    shooterIsOn = true;
    // save the targets for at goal calcs
    this.upperRPM_target = upperRPM_target;
    this.lowerRPM_target = lowerRPM_target; 
   
    upper_shooter.setRPM(upperRPM_target);
    lower_shooter.setRPM(lowerRPM_target);
  }

  public void shooterOnPerc(double upperRPM_target, double lowerRPM_target) {
    shooterIsOn = true;
      upper_shooter.setPercent(upperRPM_target); 
      lower_shooter.setPercent(lowerRPM_target); 
  }

  public boolean shooterIsOn() {
    return shooterIsOn;
  }

  public void shooterOff() {
    shooterIsOn = false;
    upper_shooter.setPercent(0.0);
    lower_shooter.setPercent(0.0);
  }

  public void magazineUp() {
    // intake will come down much faster than magazine will go up
    // so no delay here should be OK... famous last words.
    if  (isIntakeUp()) { lowerIntake();}
    magSolenoid.set(Value.kForward);
  }

  public void magazineDown() {
    magSolenoid.set(Value.kReverse);
  }

  public boolean isMagazineUp() {
    return (magSolenoid.get() == Value.kForward);
  }

  /**
   * getShooterAvgRPM()  - average of upper and lower wheels
   * @return
   */
  public double getShooterAvgRPM() {    
    return (upperRPM+lowerRPM)*0.5;
  }

  /**
   * getUpperRPM(), getLowerRPM()
   *   gets value measures in periodic() call
   * 
   * @return
   */
  public double getUpperRPM() {return upperRPM;}
  public double getLowerRPM() {return lowerRPM;}

  /**
   * getUpperTargetRPM(), getLowerTargetRPM()
   *  
   * @return last commanded upper/lower Flywheel target RPM
   */
  public double getUpperTargetRPM() {return upperRPM_target;}
  public double getLowerTargetRPM() {return lowerRPM_target;}


  /**
   * Checks to see if both flywheels are at the desired speed
   * All goals are given in Flywheel RPM.
   * 
   * Both flywheels use positive commands.
   * 
   * @param upperGoal
   * @param lowerGoal
   * @param tol
   * @return
   */
  public boolean atGoalRPM(double tol) {
    // return true if BOTH upper and lower are within tolerance 
    // Convert from passed flywheel RPMs to motor RPMs
    //System.out.println("Upper Goal:" + upperGoal + ", UpperRPM: " + upperRPM);
    //System.out.println("Lower Goal:" + lowerGoal + ", lowerRPM: " + lowerRPM +"/n");
    return ((Math.abs(upperRPM_target - upperRPM) < (upperRPM_target*tol)) &&
            (Math.abs(lowerRPM_target - lowerRPM) < (lowerRPM_target*tol)) );
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
    SmartDashboard.putNumber("/Shooter/U-MO", upper_shooter.getMotorOutputPercent());
    SmartDashboard.putNumber("/Shooter/L-MO", lower_shooter.getMotorOutputPercent());
    SmartDashboard.putNumber("/Shooter/U-RPM", upperRPM);
    SmartDashboard.putNumber("/Shooter/L-RPM", lowerRPM);
  }

/**
 *  Flywheel handles motor and gearing for the shooter flywheels.
 * 
 */
  public class FlyWheel {
    //Talon Slot stuff, we just use slot 0
    final int kSlotIdx = 0;
    final int kPIDLoopIdx = 0;
    final int kTimeoutMs = 30;

    TalonSRXConfiguration config;
    WPI_TalonSRX motor;     //this could be a generic motor controller...
    PIDFController pid;     // values used to send to hardware and tie to display, never calculate()
    
    double maxRPM;          // measured at the flywheel (tach - RPM)
    double gearRatio;       // motor turns per flywheel turn
    double FWrpm2Counts;    // flywheel RPM given motor-unit counts (f(gear, meas-period))
    double MUCounts2FWrpm;  // motor units (counts/100ms) to FW RPM (1/FWrpm2Counts)

    FlyWheel(int CAN_ID, PIDFController pidValues, boolean sensorPhase, double max_rpm) {
      config = new TalonSRXConfiguration();
      motor = new WPI_TalonSRX(CAN_ID);
      pid = new PIDFController(pidValues.getP(), pidValues.getI(), pidValues.getD(), pidValues.getF());
      maxRPM = max_rpm;

      ErrorCode lasterr = motorConfig(sensorPhase);
      if (lasterr.value != 0 ) {
        System.out.println("Flywheel motor error:" + lasterr.value + "  CANID=" + CAN_ID);
      }
    }

    ErrorCode motorConfig(boolean sensorPhase) {
      /* Factory Default all hardware to prevent unexpected behaviour */
      motor.configFactoryDefault();
      
      // use the config to set all values at once
      config.slot0.kP = pid.getP();
      config.slot0.kI = pid.getI();
      config.slot0.kD = pid.getD();
      config.slot0.kF = pid.getF();

      config.slot1 = config.slot0;
      motor.configAllSettings(config);

      /* Config sensor used for Primary PID [Velocity] */
      //motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
      // dpl - 12/19/20  Relative encoding worked very well.  See plots in Slack.
      //
      motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
      motor.setSensorPhase(sensorPhase);   // fix feedback direction
      motor.setNeutralMode(NeutralMode.Coast);

      /* Config the peak and nominal outputs */
      motor.configNominalOutputForward(0, kTimeoutMs);
      motor.configNominalOutputReverse(0, kTimeoutMs);
      motor.configPeakOutputForward(1, kTimeoutMs);
      motor.configPeakOutputReverse(-1, kTimeoutMs);
      return motor.getLastError();
    }

    /**
     * sets the gear ratio between Flywheel and motor shaft
     * @param gear
     */
    public void setMotorTurnsPerFlywheelTurn(double gear) {
      gearRatio = gear;
      // flywheel RPM given motor-unit counts (f(gear, meas-period))
      FWrpm2Counts = kRPM2Counts * gearRatio;  //motor counts are bigger, motor spins faster than FW
      
      // motor units (counts/100ms) to FW RPM 
      MUCounts2FWrpm  = 1.0 / FWrpm2Counts; 
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
      pid.setSetpoint(sp);  // saving value for possible pid display
      motor.set(ControlMode.Velocity, sp);
    }

    public void setPercent(double pct) {
      pid.setSetpoint(pct);  // saving value for possible pid display
      motor.set(ControlMode.PercentOutput, pct); 
    }

    public void setInverted() {
      motor.setInverted(true);
    }

  } //FlyWheel
}
