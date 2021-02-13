// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogIn;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PWM;

public class Magazine_Subsystem extends SubsystemBase {
  // sparkmax config
  private final boolean kInverted = false;
  private final IdleMode kIdlemode = IdleMode.kBrake;
  private final int kpidSlot =0;

  //
  final int MAG_FULL_COUNT = 3;

  //the FFHoldVolts might be useful to balance the shocks
  private double kArbFFHoldVolts = 0.0;

  // constants
  final double kCountsPerRev =  4096.0;
  final double kAngleGearRatio = 48.0;  //motor turns to pully turns
  final double kPullyDiameter = 2.0;    //inches
  final double kLengthPerMotorCount = kPullyDiameter / (kAngleGearRatio*kCountsPerRev);
  final double kLengthPerPotCount   = 1.0;
  
  final double kPotCounts2Deg = .123456;  //TODO: fix all these
  final double kMotorCounts2Deg =.012345;
  final double MAG_MIN_ANGLE = 22.0;
  final double MAG_UP_ANGLE = 29.0;      //Above this, mag is up and may interfere with intake
  final double MAG_MAX_ANGLE = 51.0;

  // magazine IO
  final Spark beltMotor = new Spark(PWM.MAGAZINE);
  final CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
  final CANPIDController anglePID = angleMotor.getPIDController();
  final DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);
  DigitalInput lightGatePwr = new DigitalInput(DigitalIO.MAGAZINE_GATE_PWR);
  AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);

  // Intake and Mag must talk, keep a reference
  Intake_Subsystem intake;

  // command values, used in periodic()
  double m_angleSetpoint;    //degrees

  // measurements update in periodic()
  double m_angle = 0.0;
  double m_angleMotor = 0.0;
  int m_pcCount = 0;


  /** Creates a new Magazine_subsystem. */
  public Magazine_Subsystem(Intake_Subsystem intake) {
    this.intake = intake;

    // fill out dashboard stuff
    SendableRegistry.setSubsystem(this, "Magazine");
    SendableRegistry.setName(anglePot, this.getName(), "Mag Angle");
    SendableRegistry.setName(lightGate, this.getName(), "Mag LightGate");
    SendableRegistry.setName(lightGatePwr, this.getName(), "Mag LightGatePwr");
    SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");
    SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");

    configureController();
    periodic();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //read angle via pot and motor encoder
    m_angle = anglePot.getValue()*kPotCounts2Deg;
    m_angleMotor = angleMotor.getEncoder().getPosition() * kMotorCounts2Deg;

    
  }

  void configureController() {
    angleMotor.restoreFactoryDefaults(false);
    angleMotor.setInverted(kInverted);
    angleMotor.setIdleMode(kIdlemode);
    angleMotor.burnFlash();
  }


  public void beltOn(double motorStrength) {
    beltMotor.set(motorStrength);
  }

  public void beltOff() {
    beltMotor.set(0);
  }

  public void raise() {
    // intake will come down much faster than magazine will go up
    // so no delay here should be OK... famous last words.
    if (intake.isIntakeUp()) {
      intake.lowerIntake();
    }
    // start moving the mag to the desired angle
    setAngle(MAG_MAX_ANGLE);
  }

  public void lower() {
    setAngle(MAG_MIN_ANGLE);
  }

  public boolean isUp() {
    return (getAngle() > MAG_UP_ANGLE) ? true : false;
  }

  public boolean isGateBlocked() {
      return lightGate.get();
  }

  public void addPC() {m_pcCount++;}
  public void removePC() {
    if (m_pcCount>0) m_pcCount--;
  }
  public int  getPC() {return m_pcCount;}
  public boolean isMagFull() { return (m_pcCount >= MAG_FULL_COUNT);}
  
  public double getAngle() {
    return m_angle;   //not sure if we want pot or motor angle
  }

  public void setAngle(double degrees) {
    m_angleSetpoint = degrees;
    anglePID.setReference(m_angleSetpoint, ControlType.kPosition, kpidSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
    
  }

}
