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
import frc.robot.util.misc.Gains;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

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

  final int kSlotIdx = 0;
  final int kPIDLoopIdx = 0;
  final int kTimeoutMs = 30;
  
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
  Gains kGains_Velocit = new Gains(16.0, 0.001, 0.0, 0.0, 300, 1.00);

  /**
   * Convert Target RPM to units / 100ms. 4096 Units/Rev * Target RPM * 600 =
   * velocity setpoint is in units/100ms
   * 
   * Gear ratio is 10:1 and sensor is after the gears
   */

  final double GEAR = 10.; // 10:1 gear, encoder after the gears
  final double ShooterEncoder = 4096; // counts per rev
  final double RPM2CountsPer100ms = 600.0; // Vel uses 100mS as counter sample period
  //final double kRPM2Counts = (GEAR * ShooterEncoder) / RPM2CountsPer100ms;
  final double kRPM2Counts = 1.611328125; // (60s / 409.6 which is 1 RPS, seems more reasonable with 7000 RPMs now at full output)
  
  private double lowerRPM;
  private double upperRPM;
  private double targetRPM;

  private boolean intakeIsOn;
  private boolean shooterIsOn;

  private boolean percentControlled;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
  private NetworkTableEntry test =
     tab.add("test value", 1)
        .getEntry();

  public Intake_Subsystem(boolean percentControlled) {
    this.percentControlled = percentControlled;
    if(!percentControlled){
      shooterMotorConfig(upper_shooter);
      shooterMotorConfig(lower_shooter);
    }
    
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
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);

    talon.configAllSettings(shooterCfg);
    talon.setNeutralMode(NeutralMode.Brake);
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

  public void shooterOn(double UpperRPM_target, double LowerRPM_target) {
    shooterIsOn = true;
    targetRPM = RPM_target;

    /*
     * Velocity Closed Loop double targetVelocity_UnitsPer100ms = RPM_target *
     * kRPM2Counts;
    */
    if(percentControlled){
      upper_shooter.set(ControlMode.PercentOutput, RPM_target); //for percent control
      lower_shooter.set(ControlMode.PercentOutput, RPM_target); 
    }
    else{
      upper_shooter.set(ControlMode.Velocity, RPM_target*kRPM2Counts);
      lower_shooter.set(ControlMode.Velocity, -RPM_target*kRPM2Counts);
    }
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


  public double getShooterRPM() {    
    return (upperRPM+lowerRPM)/2;
  }

  public boolean atGoalRPM(double upperGoal, double lowerGoal, double tol){
    // return true if BOTH upper and lower are within tolerance
    return ((upperGoal-upperRPM < (upperGoal*tol)) && (lowerGoal-lowerRPM < (lowerGoal*tol)));
  }

  public double getShooterPercent() {
    // Get the current output percent of the upper and lower shooter motors
    double upperPerc = upper_shooter.getMotorOutputPercent();
    double lowerPerc = lower_shooter.getMotorOutputPercent();
    // Gets the lower of the upper and lower shooter current speed
    //dpl may want average here?
    return Math.min(upperPerc, lowerPerc);
  }

  @Override
  public void log() {
    // Put any useful log message here, called about 10x per second
    double upperVelocity = upper_shooter.getSelectedSensorVelocity();
    double lowerVelocity = lower_shooter.getSelectedSensorVelocity();
    upperRPM = upperVelocity/kRPM2Counts;
    lowerRPM = -lowerVelocity/kRPM2Counts;

    SmartDashboard.putNumber("Upper Shooter Percent", upper_shooter.getMotorOutputPercent());
    SmartDashboard.putNumber("Lower Shooter Percent", lower_shooter.getMotorOutputPercent());
    SmartDashboard.putNumber("Upper Shooter RPM", upperRPM);
    SmartDashboard.putNumber("Lower Shooter RPM", lowerRPM);
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    SmartDashboard.putNumber("Shooter Velocity Target", targetRPM); // Logs the target velocity
    SmartDashboard.putNumber("Input test", test.getDouble(1.0));
  }
}
