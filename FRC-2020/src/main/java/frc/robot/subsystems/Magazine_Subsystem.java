// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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
import frc.robot.util.misc.MathUtil;

public class Magazine_Subsystem extends SubsystemBase {
  // Physical limits
  static final double MAG_MIN_ANGLE = 22.0;
  static final double MAG_UP_ANGLE = 29.0; // Above this, mag is up and may interfere with intake
  static final double MAG_MAX_ANGLE = 55.0;
  // arm lengths for linear pot anchor points
  static final double MAG_UPPER_LEN = 16.25;
  static final double MAG_LOWER_LEN = 11.5; // inch

  /**
   * MagazinePosition_Subsystem
   * 
   * Setup so it can run its own commands.
   * 
   */
  public class MagazinePosition extends SubsystemBase {
    // sparkmax config
    private final boolean kInverted = false;
    private final IdleMode kIdlemode = IdleMode.kBrake;
    private final int kpidSlot = 0;

    // the FFHoldVolts might be useful to balance the shocks
    private double kArbFFHoldVolts = 0.0;

    // motor constants & linear pot
    static final double kCountsPerRev = 4096.0;
    static final double kAngleGearRatio = 48.0; // motor turns to pully turns
    static final double kPullyDiameter = 2.0; // inches
    static final double kMotorInchPerCount = kPullyDiameter / (kAngleGearRatio * kCountsPerRev);

    // Pot Volts measured at top & bottom position
    static final double Vat22 = 0.650; // volts at 22 degrees (min angle)
    static final double Vat55 = 3.707; // volts at 55 degrees (max angle)

    /**
     * Pot reading reprents length which is not linar because we have the sensor
     * mounted at different radi on the mag.
     * 
     * b^2 + c^2 - l^2 cos( 55 - 22) = -------------------- bc
     * 
     * Need to use law of cosines to determine L at angles
     */
    static final double b2 = MAG_LOWER_LEN * MAG_LOWER_LEN;
    static final double c2 = MAG_UPPER_LEN * MAG_UPPER_LEN;
    static final double bc = MAG_UPPER_LEN * MAG_LOWER_LEN;
    static final double b2c2_bc = (b2 + c2) / bc;
    final double POT_LENGTH_min = Math.sqrt(b2 + c2 - bc * Math.cos(Math.toRadians(22)));
    final double POT_LENGTH_max = Math.sqrt(b2 + c2 - bc * Math.cos(Math.toRadians(55)));
    final double kLengthPerVolt = (POT_LENGTH_max - POT_LENGTH_min) / (Vat55 - Vat22);

    // postion control devices
    final CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
    final CANEncoder angleEncoder = angleMotor.getEncoder();
    final CANPIDController anglePID = angleMotor.getPIDController();
    final AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);

    // measurements
    double m_pot_length;
    double m_angle = 0.0;
    double m_angleMotor = 0.0;

    // command values, used in periodic()
    double m_lengthSetpoint; // inches

    MagazinePosition() {
      SendableRegistry.setName(anglePot, this.getName(), "Mag Angle");
      // configure sparkmax motor
      angleMotor.restoreFactoryDefaults(false);
      angleMotor.setInverted(kInverted);
      angleMotor.setIdleMode(kIdlemode);
      angleMotor.burnFlash();

    }

    @Override
    public void periodic() {
      // read angle via pot and motor encoder
      m_pot_length = anglePot.getValue() * kLengthPerVolt;
      double l2 = m_pot_length * m_pot_length;
      m_angle = Math.toRadians(Math.acos(b2c2_bc - l2 / bc));

      // nw calc the angle based on the motor lenght
      double motor_l = angleEncoder.getPosition() * kMotorInchPerCount;
      l2 = motor_l * motor_l;
      m_angleMotor = Math.toRadians(Math.acos(b2c2_bc - l2 / bc));
    }

    double getPotAngle() {
      return m_angle;
    }

    double getMotorAngle() {
      return m_angleMotor;
    }

    double getAngle() {
      return m_angle;
    }

    public void setAngle(double degrees) {
      double deg = MathUtil.limit(degrees, MAG_MIN_ANGLE, MAG_MAX_ANGLE);
      m_lengthSetpoint = Math.sqrt(b2 + c2 - bc * Math.cos(Math.toRadians(deg)));
      anglePID.setReference(m_lengthSetpoint, ControlType.kPosition, kpidSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
    }
  }

  /**
   * 
   */
  final int MAG_FULL_COUNT = 3;

  // magazine Belt
  final Spark beltMotor = new Spark(PWM.MAGAZINE);

  final DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);

  // Intake and Mag must talk, keep a reference
  Intake_Subsystem intake;
  MagazinePosition magPostion;

  // measurements update in periodic()
  int m_pcCount = 0;

  /** Creates a new Magazine_subsystem. */
  public Magazine_Subsystem(Intake_Subsystem intake) {
    this.intake = intake;
    this.magPostion = new MagazinePosition();

    // fill out dashboard stuff
    SendableRegistry.setSubsystem(this, "Magazine");
    SendableRegistry.setName(lightGate, this.getName(), "Mag LightGate");
    SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");
  }

  @Override
  public void periodic() {

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
    magPostion.setAngle(MAG_MAX_ANGLE);
  }

  public void lower() {
    magPostion.setAngle(MAG_MIN_ANGLE);
  }

  public boolean isUp() {
    return (magPostion.getAngle() > MAG_UP_ANGLE) ? true : false;
  }

  /**
   * Lightgate has a 2K pull down resister on it. Signal goes high when the light
   * is broken.
   * 
   * https://cdn.automationdirect.com/static/specs/pe18mmthroughbeamfb.pdf
   * 
   * @return true when blocked by power cell.
   */
  public boolean isGateBlocked() {
    return lightGate.get();
  }

  public void addPC() {
    m_pcCount++;
  }

  public void removePC() {
    if (m_pcCount > 0)
      m_pcCount--;
  }

  public int getPC() {
    return m_pcCount;
  }

  public boolean isMagFull() {
    return (m_pcCount >= MAG_FULL_COUNT);
  }

  public double getAngle() {
    return magPostion.getAngle();
  }

}
