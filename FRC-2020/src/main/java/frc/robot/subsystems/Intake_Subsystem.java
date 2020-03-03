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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.*;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.util.misc.Gains;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * motorStrength should be between -1 and 1 for all methods
 */
public class Intake_Subsystem extends SubsystemBase implements Logger {
  /**
   * Creates a new Intake_Subsystem.
   * 
   * Use the WPI_TalonSRX wrapper around the lower level TalonSrx because it
   * implements the WPI SpeedController, Sendable.
   * 
   * 
   * Who When What DPL 2/09/2020 removed publics, use WPI_TalonSRX, Gear gain Also
   * removed some debug code.
   * 
   */

  // Intake
  Spark intake_spark = new Spark(INTAKE_SPARK_PWM);

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(INTAKE_PCM_CAN_ID, INTAKE_UP_SOLENOID_PCM,
      INTAKE_DOWN_SOLENOID_PCM);

  DoubleSolenoid magSolenoid = new DoubleSolenoid(MAGAZINE_PCM_CAN_ID, MAGAZINE_UP_PCM, MAGAZINE_DOWN_PCM);

  // magazine
  Spark magazine = new Spark(MAGAZINE_PWM);

  // shooters
  WPI_TalonSRX upper_shooter = new WPI_TalonSRX(UPPER_SHOOTER_TALON_CAN);
  WPI_TalonSRX lower_shooter = new WPI_TalonSRX(LOWER_SHOOTER_TALON_CAN);

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
   * or 3. Only the first two (0,1) are visible in web-based configuration.
   */
  final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
   * we just want the primary one.
   */
  final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  final int kTimeoutMs = 30;

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
   * Velocity units at 100% output
   * 
   * kP kI kD kF Iz PeakOut
   */
  // final static Gains kGains_Velocit = new Gains(0.25, 0.001, 20, 1023.0 /
  // 7200.0, 300, 1.00);
  ErrorCode lastError;
  TalonSRXConfiguration shooterCfg = new TalonSRXConfiguration();

  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 7200 represents
   * Velocity units at 100% output
   * 
   * DPL - shouldn't need a kF unless the friction is very high and needs to be
   * overcome. - kD zero is a good start
   * 
   * kP kI kD kF Iz PeakOut
   */
  Gains kGains_Velocit = new Gains(1.0, 0.001, 0.0, 0.0, 300, 1.00);

  /**
   * Convert Target RPM to units / 100ms. 4096 Units/Rev * Target RPM * 600 =
   * velocity setpoint is in units/100ms
   * 
   * Gear ratio is 10:1 and sensor is after the gears
   */

  final double GEAR = 10.; // 10:1 gear, encoder after the gears
  final double ShooterEncoder = 4096; // counts per rev
  final double RPM2CountsPer100ms = 600.0; // Vel uses 100mS as counter sample period
  final double kRPM2Counts = (GEAR * ShooterEncoder) / RPM2CountsPer100ms;

  private boolean intakeIsOn;
  private boolean shooterIsOn;

  public Intake_Subsystem() {
    shooterMotorConfig(upper_shooter);
    shooterMotorConfig(lower_shooter);

    /* Config the Velocity closed loop gains in slot0 */
    // upper_shooter_talon.config_kF(kPIDLoopIdx, 0.0 /*kGains_Velocit.kF */,
    // kTimeoutMs);
    // upper_shooter_talon.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
    // upper_shooter_talon.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
    // upper_shooter_talon.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);

    intakeIsOn = false;
    shooterIsOn = false;

    magazineDown(); // must start in down positon
    raiseIntake();  // must start in the up position
  }

  void shooterMotorConfig(WPI_TalonSRX talon) {
    /* Factory Default all hardware to prevent unexpected behaviour */
    talon.configFactoryDefault();

    // use the config to set all values at once
    shooterCfg.slot0.kP = kGains_Velocit.kP;
    shooterCfg.slot0.kI = kGains_Velocit.kI;
    shooterCfg.slot0.kD = kGains_Velocit.kD;
    shooterCfg.slot0.kF = kGains_Velocit.kF;

    shooterCfg.slot1 = shooterCfg.slot0;

    /* Config sensor used for Primary PID [Velocity] */
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

    talon.configAllSettings(shooterCfg);

    /* Config the peak and nominal outputs */
    talon.configNominalOutputForward(0, kTimeoutMs);
    talon.configNominalOutputReverse(0, kTimeoutMs);
    talon.configPeakOutputForward(1, kTimeoutMs);
    talon.configPeakOutputReverse(-1, kTimeoutMs);
    lastError = talon.getLastError();
  }

  @Override
  public void periodic() {
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

  public void shooterOn(double RPM_target) {
    shooterIsOn = true;
    /*
     * Velocity Closed Loop double targetVelocity_UnitsPer100ms = RPM_target *
     * kRPM2Counts;
     */
    // WIP - using simple motorpercent for now - 2/13/20
    upper_shooter.set(ControlMode.PercentOutput, RPM_target);
    lower_shooter.set(ControlMode.PercentOutput, RPM_target);
  }

  public boolean shooterIsOn() {
    return shooterIsOn;
  }

  public void shooterOff() {
    shooterIsOn = false;
    upper_shooter.set(ControlMode.PercentOutput, 0);
    lower_shooter.set(ControlMode.PercentOutput, 0);
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

  @Override
  public void log() {
    // Put any useful log message here, called about 10x per second

  }

}
