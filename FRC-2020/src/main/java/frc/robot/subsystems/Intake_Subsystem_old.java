/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class Intake_Subsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.




  //Intake
  public WPI_TalonSRX intake_talon = new WPI_TalonSRX(Constants.INTAKE_TALON_CAN);
  
  public DoubleSolenoid elevatorSolenoid = new DoubleSolenoid(Constants.ELEVATOR_PCM_ID, 
          Constants.ELEVATOR_UP_SOLENOID_PCM, Constants.ELEVATOR_DOWN_SOLENOID_PCM);
  
  //Intake Pneumatic Sensors
  public DigitalInput intake_up_sensor = new DigitalInput(Constants.INTAKE_UP_DIO);
  public DigitalInput intake_down_sensor = new DigitalInput(Constants.INTAKE_DOWN_DIO);

  //magazine
  public WPI_TalonSRX magazine_talon = new WPI_TalonSRX(Constants.MAGAZINE_TALON_CAN);
  //shooters
  public WPI_TalonSRX upper_shooter_talon = new WPI_TalonSRX(Constants.UPPER_SHOOTER_TALON_CAN);
  public WPI_TalonSRX lower_shooter_talon = new WPI_TalonSRX(Constants.LOWER_SHOOTER_TALON_CAN);
  //elevator
  public WPI_TalonSRX elevator_talon = new WPI_TalonSRX(Constants.ELEVATOR_TALON_CAN);




/*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  */

  public Intake_Subsystem(){

  }

  public void intakeUp() {
    elevatorSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeDown() {
    elevatorSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isIntakeUp(){
    //DIO
    return intake_up_sensor.get();
  }

  public boolean isIntakeDown(){
    //DIO
    return intake_down_sensor.get();
  }

  public void intakeOn(){
    intake_talon.set(0.2);
  }

  
  public void intakeOff(){
    intake_talon.set(0.0);
  }



  //method to adjust intake elevator

  public void magazineOn(){
    magazine_talon.set(0.2);
  }

  
  public void magazineOff(){
    magazine_talon.set(0);
  }

  public void upperShooterOn(){
    upper_shooter_talon.set(0.2);
  }

  
  public void upperShooterOff(){
    upper_shooter_talon.set(0);
  }

  
  public void lowerShooterOn(){
    lower_shooter_talon.set(0.2);
  }

  public void lowerShooterOff(){
    lower_shooter_talon.set(0);
  }

  public void elevatorOn(){
    elevator_talon.set(0.2);
  }


}
