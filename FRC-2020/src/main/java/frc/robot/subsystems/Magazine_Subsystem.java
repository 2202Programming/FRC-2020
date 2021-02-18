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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AnalogIn;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PCM2;
import frc.robot.Constants.PWM;
import frc.robot.util.misc.MathUtil;
import frc.robot.util.misc.PIDFController;

public class Magazine_Subsystem extends SubsystemBase {
  // Physical limits
  static final double MIN_ANGLE = 22.0;
  static final double UP_ANGLE = 29.0; // Above this, mag is up and may interfere with intake
  static final double MAX_ANGLE = 55.0;

  /**
   * See this doc for calcs and numbers
   * 
   * https://docs.google.com/spreadsheets/d/1-3pbHYxEgDEzljsYpuX3GjYhh8UF8N46/edit#gid=171427340
   * 
   */
  static final double MAG_ANGLE_OFFSET = 28.07; //degrees - rotation due to frame

  // arm lengths for linear pot anchor points
  static final double POT_UPPER_LEN = 18.783; // inch
  static final double POT_LOWER_LEN = 18.18;  // inch
  static final double POT_OFFSET_E = 15.67;   //degres - off set from pot angle base

  // Strap / motor lengths - includes pully
  static final double STRAP_UPPER_LEN = 22.183;  // inch
  static final double STRAP_LOWER_LEN = 20.22;   // inch
  static final double STRAP_OFFSET_E = 19.99;    //degres - off set from pot angle base

  //law of cosine constants for motor strap calcs
  static final double m_b2 = STRAP_LOWER_LEN * STRAP_LOWER_LEN;
  static final double m_c2 = STRAP_UPPER_LEN * STRAP_UPPER_LEN;
  static final double m_bc2 = 2.0*STRAP_UPPER_LEN * STRAP_LOWER_LEN;
  static final double m_b2c2_2bc = (m_b2 + m_c2) / (m_bc2);
  static final double STRAP_LENGTH_min = Math.sqrt(m_b2 + m_c2 - m_bc2 * 
      Math.cos(Math.toRadians(MIN_ANGLE - (MAG_ANGLE_OFFSET - STRAP_OFFSET_E) )));

  /**
   * MagazinePositioner_Subsystem
   * 
   * Setup so it can run its own commands.
   * 
   */
  public class MagazinePositioner extends SubsystemBase {
    // sparkmax config
    private final boolean kInverted = false;
    private final IdleMode kIdlemode = IdleMode.kCoast;
    private final int kpidSlot = 0;

    //pid values for motor P, I, D, F
    PIDFController pidvalues = new PIDFController(0.1, 0.0, 0.0, 0.0);
    {pidvalues.setIzone(0.0); }
  
    // the FFHoldVolts might be useful to balance the shocks
    private double kArbFFHoldVolts = 0.0;


    // motor constants & linear pot
    static final double kGearRatio = 48.0; // motor turns to pully turns
    static final double kPullyDiameter = 1.4;   // inches (1in without belting)
    static final double kInchPerMotorRev = Math.PI*kPullyDiameter / kGearRatio;
    static final double kRevPerInch = 1.0 / kInchPerMotorRev;
    static final double kMinVelZeroTol = .01;  //inches/sec
    static final double kMaxRPM = 30;

    // Pot Volts measured at top & bottom position
    static final double VatMin = 0.650; // volts at 22 degrees (min angle)
    static final double VatMax = 3.707; // volts at 55 degrees (max angle)
    static final double PotLengthMin = 3.15; // inch compare with Law of Cosine
    static final double PotLengthMax = 13.44; // inch

    static final double kToleranceDeg = 0.1;

    /**
     * Pot reading reprents length which is not linar because we have the sensor 
     * mounted at different radi on the mag.  (it is very close now.)
     * 
     * Need to use law of cosines to determine L at angles
     */
    static final double b2 = POT_LOWER_LEN * POT_LOWER_LEN;
    static final double c2 = POT_UPPER_LEN * POT_UPPER_LEN;
    static final double bc_2 = 2.0*POT_UPPER_LEN * POT_LOWER_LEN;
    static final double b2c2_bc = (b2 + c2) / (bc_2);
    final double POT_LENGTH_min = Math.sqrt(b2 + c2 - bc_2 * Math.cos(Math.toRadians(MIN_ANGLE - (MAG_ANGLE_OFFSET - POT_OFFSET_E) )));
    final double POT_LENGTH_max = Math.sqrt(b2 + c2 - bc_2 * Math.cos(Math.toRadians(MAX_ANGLE - (MAG_ANGLE_OFFSET - POT_OFFSET_E))));
    final double kInchPerVolt = (POT_LENGTH_max - POT_LENGTH_min) / (VatMax - VatMin);
    final double kDegPerVolt = ((42.6 - 9.6 /*MAX_ANGLE - MIN_ANGLE*/)/(VatMax - VatMin));

    // postion control devices
    final CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
    final CANEncoder angleEncoder = angleMotor.getEncoder();
    final CANPIDController anglePID = angleMotor.getPIDController();
    final AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);
    final DoubleSolenoid magSolenoid = new DoubleSolenoid(CAN.PCM2, PCM2.MAG_LOCK, PCM2.MAG_UNLOCK);
  
    // measurements
    double m_pot_length;
    double m_anglePot = 0.0;
    double m_angle_LOC = 0.0;     //exact calc to compare with estimated
    double m_angleMotor = 0.0;
    double m_strapSpeed = 0.0;  //inches/s
    double m_angle;  //actual angle in robot coordinates

    // command values, used in periodic()
    double m_lengthSetpoint; // inches  <calculated from angleSetpoint>
    double m_angleSetpoint;  // degrees <input>

    MagazinePositioner() {
      SendableRegistry.setName(anglePot, this.getName(), "Mag Angle");
      
      // configure sparkmax motor
      angleMotor.restoreFactoryDefaults(false);
      angleMotor.setInverted(kInverted);
      angleMotor.setIdleMode(kIdlemode);
     
      //copy the sw pidvalues to the hardware
      pidvalues.copyTo(anglePID, kpidSlot);
      angleMotor.burnFlash();
      angleEncoder.setPosition(0.0);
     
      stop();  //this locks
      unlock();
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
      layout.addNumber("MAGPos/angle", () -> m_angle).withSize(2, 5);
      layout.addNumber("MAGPos/Strap_Speed", () -> m_strapSpeed);
      layout.addNumber("MAGPos/pot_len", () -> m_pot_length);
      layout.addNumber("MAGPos/angle_mo", () -> m_angleMotor);
    }


    @Override
    public void periodic() {
      // read pot to get length
      double apv =anglePot.getAverageVoltage();
      m_pot_length = POT_LENGTH_min + apv * kInchPerVolt;
      double l2 = m_pot_length * m_pot_length;
      m_angle_LOC = Math.toRadians(Math.acos(b2c2_bc - l2 / bc_2));

      //linear estimate should be good enought based on spreadsheet 
      m_anglePot = kDegPerVolt * apv;

      // nw calc the angle based on the motor lenght
      double motor_l = STRAP_LENGTH_min + angleEncoder.getPosition() * kInchPerMotorRev;
      l2 = motor_l * motor_l;
      m_angleMotor = Math.toRadians(Math.acos(m_b2c2_2bc - l2 /m_bc2));

      // use pot to give angle in robot frame
      m_angle = m_anglePot + (MAG_ANGLE_OFFSET - POT_OFFSET_E);

      //measure strap speed
      m_strapSpeed = angleEncoder.getVelocity()*(kInchPerMotorRev/60.0);

      safety();
    }

    double getPotAngle() {
      return m_angle;
    }

    double getMotorAngle() {
      return m_angleMotor;
    }

    /**
     * gets Magazine angle in robot coordinates
     * @return
     */
    public double get() {
      return m_angle;
    }

    public void setAngle(double magDeg) {
      magDeg = MathUtil.limit(magDeg, MIN_ANGLE, MAX_ANGLE);
      double strap_deg = magDeg - (MAG_ANGLE_OFFSET - STRAP_OFFSET_E);
      m_lengthSetpoint = Math.sqrt(m_b2 + m_c2 - m_bc2 * Math.cos(Math.toRadians(strap_deg)));
      double setpoint = (m_lengthSetpoint - STRAP_LENGTH_min)*kRevPerInch;
      
      // unlock the gear, it gets re-locked when at setpoint
      unlock();
      anglePID.setReference(setpoint, ControlType.kPosition, kpidSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
    }

    /**
     * use velocity control to wind motor
     * @param speed  RPM of takeup pully
     */
    public void wind(double pully_rpm) {
      pully_rpm = MathUtil.limit(pully_rpm,-kMaxRPM, kMaxRPM);
      double motor_speed = kGearRatio*pully_rpm;
      unlock();
      anglePID.setReference(motor_speed, ControlType.kVelocity, kpidSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
      
    }

    public void stop() {
      angleMotor.stopMotor();
      m_angleSetpoint = m_angle;  //force no error between current angle and setpoint
      lock();
    }

    public boolean isAtBottom() {
      return (m_angle <= MIN_ANGLE)  ?  true : false;
    }
    
    public boolean isAtTop() {
      return (m_angle >= MAX_ANGLE)  ?  true : false;
    }
    
    public boolean isAtSetpoint() {
      return (Math.abs(m_angle - m_angleSetpoint) < kToleranceDeg)  ? true : false;     
    }

    //checks to make sure we don't do stupid
    void safety() {
      //not sure what stupid is yet
      //maybe a current check for being against the stop?
    }

    /**
     * controls for the lock on the 
     */
    public void lock() {
      magSolenoid.set(Value.kForward);
    }
  
    public void unlock() {
      magSolenoid.set(Value.kReverse);
    }
  
    public boolean isLocked() {
      return (magSolenoid.get() == Value.kForward);
    }

    public boolean isMoving() {
      return (Math.abs(m_strapSpeed) > kMinVelZeroTol) ? true : false;
    }

  } // MagazinePositioner
  /**
   * 
   */
  final int MAG_FULL_COUNT = 3;

  // magazine Belt
  final Spark beltMotor = new Spark(PWM.MAGAZINE);

  final DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);

  // Intake and Mag must talk, keep a reference
  Intake_Subsystem intake;
  MagazinePositioner positioner;

  // measurements update in periodic()
  int m_pcCount = 0;

  /** Creates a new Magazine_subsystem. */
  public Magazine_Subsystem(Intake_Subsystem intake) {
    this.intake = intake;
    this.positioner = new MagazinePositioner();

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
    positioner.setAngle(MAX_ANGLE);
  }

  public void lower() {
    positioner.setAngle(MIN_ANGLE);
  }

  //TODO: remove this function
  public boolean isUp() {
    return (positioner.get() > UP_ANGLE) ? true : false;
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

  public void setPC(int c) {
    m_pcCount = (c >= 0  && c <= MAG_FULL_COUNT) ? c : 0;
  }
  public boolean isMagFull() {
    return (m_pcCount >= MAG_FULL_COUNT);
  }

  public double getAngle() {
    return positioner.get();
  }

  public MagazinePositioner getMagPositioner() {
    return positioner;
  }



}
