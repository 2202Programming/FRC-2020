// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;

public class Climber extends  MonitoredSubsystemBase implements Logger {

  final double CLIMB_SPEED = .60;
  final DoubleSolenoid.Value EXTEND  = Value.kForward;
  final DoubleSolenoid.Value RETRACT = Value.kReverse;
  
 
  final WPI_TalonSRX motor = new WPI_TalonSRX(CAN.CLIMBER_TALON);  
  final TalonSRXConfiguration srxconfig = new TalonSRXConfiguration();
  final DoubleSolenoid solenoid = new DoubleSolenoid(CAN.PCM1, PCM1.CLIMBER_EXTEND, PCM1.CLIMBER_RETRACT);
  
  public Climber() {

    // add some current limts to try to protext motor
    motor.configFactoryDefault();
    motor.configContinuousCurrentLimit(25); //amps (max is around 30 amps @ 12V)
    motor.configPeakCurrentLimit(100);      //amps (130 is stated stall of redline on Andy Mark)
    motor.configPeakCurrentDuration(200);   //mS   (wild guess)
  

    solenoid.clearAllPCMStickyFaults();
    solenoid.set(RETRACT);
    off();
  }

  @Override
  public void monitored_periodic() {  }

  @Override
  public void log() {   }

  //Subsystem API 
  public void extendArm() {
    solenoid.set(EXTEND);
  }

  public void retractArm() {
    solenoid.set(RETRACT);
  }

  public boolean isExtended() {
    return (solenoid.get() == EXTEND);
  }

  public void off() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void climbUp() {
    motor.set(ControlMode.PercentOutput, -CLIMB_SPEED);
  }

  public void climbDown() {
    motor.set(ControlMode.PercentOutput, CLIMB_SPEED);
  }

}
