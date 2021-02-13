// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogIn;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PWM;

public class Magazine_Subsystem extends SubsystemBase {
  // constants
  final double MIN_ANGLE = 22.0;
  final double MAX_ANGLE = 51.0;

  // magazine IO
  Spark beltMotor = new Spark(PWM.MAGAZINE);
  CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
  DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);
  DigitalInput lightGatePwr = new DigitalInput(DigitalIO.MAGAZINE_GATE_PWR);
  AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);

  /** Creates a new Magazine_subsystem. */
  public Magazine_Subsystem(Intake_Subsystem intake) {
    // fill out dashboard stuff
    SendableRegistry.setSubsystem(this, "Magazine");
    SendableRegistry.setName(anglePot, this.getName(), "Mag Angle");
    SendableRegistry.setName(lightGate, this.getName(), "Mag LightGate");
    SendableRegistry.setName(lightGatePwr, this.getName(), "Mag LightGate");
    SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");
    SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //read angle
    this

  }

  public void magazineOn(double motorStrength) {
    beltMotor.set(motorStrength);
  }

  public void magazineOff() {
    beltMotor.set(0);
  }

  public void magazineUp() {
    // intake will come down much faster than magazine will go up
    // so no delay here should be OK... famous last words.
    if (isIntakeUp()) {
      lowerIntake();
    }
   
  }

  public boolean isGateBlocked() {
    lightGate.get();
  }

  public void magazineDown() {
    setAngle(MIN_ANGLE);
  }

  public double getAngle() {
    return -1.0;
  }

  public void setAngle(double degrees) {

  }

}
