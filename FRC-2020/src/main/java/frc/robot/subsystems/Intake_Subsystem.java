/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
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
  public WPI_TalonSRX upper_shooter_talon = new WPI_TalonSRX(Constants.UPPER_SHOOTER_TALON_CAN);
  //public WPI_TalonSRX lower_shooter_talon = new WPI_TalonSRX(Constants.LOWER_SHOOTER_TALON_CAN);
  // elevator
  public WPI_TalonSRX elevator_talon = new WPI_TalonSRX(Constants.ELEVATOR_TALON_CAN);

  public Intake_Subsystem() {
  
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

  public void shooterOn(double motorStrength) {
    upper_shooter_talon.set(motorStrength);
    //lower_shooter_talon.set(motorStrength);
  }

  public void shooterOff() {
    upper_shooter_talon.set(0);
    //lower_shooter_talon.set(0);
  }


}
