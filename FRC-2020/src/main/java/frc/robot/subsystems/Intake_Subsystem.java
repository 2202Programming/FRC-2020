/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Gains;
import edu.wpi.first.wpilibj.DigitalInput;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Spark;

/**
 * motorStrength should be between -1 and 1 for all methods
 */
public class Intake_Subsystem extends SubsystemBase {
  /**
   * Creates a new Intake_Subsystem.
   */

  // Intake
  public Spark intake_spark = new Spark(Constants.INTAKE_SPARK_PWM);

  public DoubleSolenoid elevatorSolenoid = new DoubleSolenoid(Constants.ELEVATOR_PCM_ID,
      Constants.ELEVATOR_UP_SOLENOID_PCM, Constants.ELEVATOR_DOWN_SOLENOID_PCM);

  // Intake Pneumatic Sensors
  //public DigitalInput intake_up_sensor = new DigitalInput(Constants.INTAKE_UP_DIO);
  //public DigitalInput intake_down_sensor = new DigitalInput(Constants.INTAKE_DOWN_DIO);

  // magazine
  public WPI_TalonSRX magazine_talon = new WPI_TalonSRX(Constants.MAGAZINE_TALON_CAN);
  // shooters
  public TalonSRX upper_shooter_talon = new TalonSRX(Constants.UPPER_SHOOTER_TALON_CAN);
  //public WPI_TalonSRX lower_shooter_talon = new WPI_TalonSRX(Constants.LOWER_SHOOTER_TALON_CAN);
  // elevator
  public WPI_TalonSRX elevator_talon = new WPI_TalonSRX(Constants.ELEVATOR_TALON_CAN);


	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
    public final static Gains kGains_Velocit = new Gains( 0.25, 0.001, 20, 1023.0/7200.0,  300,  1.00);


  public Intake_Subsystem() {
     /* Factory Default all hardware to prevent unexpected behaviour */
    upper_shooter_talon.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    upper_shooter_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    kPIDLoopIdx, kTimeoutMs);

    /* Config the peak and nominal outputs */
		upper_shooter_talon.configNominalOutputForward(0, kTimeoutMs);
		upper_shooter_talon.configNominalOutputReverse(0, kTimeoutMs);
		upper_shooter_talon.configPeakOutputForward(1, kTimeoutMs);
		upper_shooter_talon.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		upper_shooter_talon.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
		upper_shooter_talon.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
		upper_shooter_talon.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
		upper_shooter_talon.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  /*
   * 
   * public boolean isIntakeUp(){ //DIO return intake_up_sensor.get(); }
   * 
   * public boolean isIntakeDown(){ //DIO return intake_down_sensor.get(); }
   * 
   */

  public void raiseIntake(){
    elevatorSolenoid.set(kForward);
  } 

  public void lowerIntake(){
    elevatorSolenoid.set(kReverse);
  } 

  public void intakeOn(double motorStrength) {
    intake_spark.set(motorStrength);
  }

  public void intakeOff() {
    intake_spark.set(0);
  }

  public void magazineOn(double motorStrength) {
    magazine_talon.set(motorStrength);
  }

  public void magazineOff() {
    magazine_talon.set(0);
  }

  public void shooterOn(double RPM_target) {

      /* Velocity Closed Loop */

			/**
			 * Convert Target RPM to units / 100ms.
			 * 4096 Units/Rev * Target RPM * 600 = 
			 * velocity setpoint is in units/100ms
       * CHECK UNITS/REV WITH GEARBOX!
			 */
			double targetVelocity_UnitsPer100ms = RPM_target * 4096 * 600;
			/* 500 RPM in either direction */
			upper_shooter_talon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

  }

  public void shooterOff() {
    upper_shooter_talon.set(ControlMode.PercentOutput,0);
 
  }


}
