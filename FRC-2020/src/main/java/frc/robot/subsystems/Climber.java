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
import frc.robot.Constants.PCM2;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;

public class Climber extends  MonitoredSubsystemBase implements Logger {

  final double CLIMB_SPEED = .60;
 
  final TalonSRXConfiguration srxconfig = new TalonSRXConfiguration();
  final WPI_TalonSRX motor = new WPI_TalonSRX(CAN.CLIMBER_TALON);     //this could be a generic motor controller...
  final DoubleSolenoid solenoid = new DoubleSolenoid(CAN.PCM2, PCM2.CLIMBER_EXTEND, PCM2.CLIMBER_RETRACT);
  
  public Climber() {
    retractArm();
    off();
  }

  @Override
  public void monitored_periodic() {

  }

  @Override
  public void log() { 
  }

  //Subsystem API 
  public void extendArm() {
    solenoid.set(Value.kForward);
  }

  public void retractArm() {
    solenoid.set(Value.kReverse);
  }
  public boolean isExtended() {
    return (solenoid.get() == Value.kForward);
  }

  public void off() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void climbUp() {
    motor.set(ControlMode.PercentOutput, CLIMB_SPEED);
  }

  public void climbDown() {
    motor.set(ControlMode.PercentOutput, -CLIMB_SPEED);
  }

}
