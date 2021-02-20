// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DT;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
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
  static final double MAX_ANGLE = 55.0;

  static final double MAX_SOFT_STOP = 50.0;
  static final double MIN_SOFT_STOP = 25.0;

  /**
   * See this doc for calcs and numbers
   * 
   * https://docs.google.com/spreadsheets/d/1-3pbHYxEgDEzljsYpuX3GjYhh8UF8N46/edit#gid=171427340
   * 
   * 
   * Mag angle = position from horizontal ie the shooter angle (what we really use)
   * Strap angle = angle opposite the motor strap  (what we have from the Neo)
   * Pot angle = angle opposite the yo-yo pot  (what we have from the pot sensor)
   * 
   * xxx_OFFSET_E refers to the angle E in the above dod for Strap or Pot and is
   * the rotation needed to get into the Mag's coordinates.
   * 
   * MAG_ANGLE_OFFSET gets us to the degrees from horizontal we need.
   * 
   */
  static final double MAG_ANGLE_OFFSET = 28.07; //degrees - rotation due to frame

  // arm lengths for yo-yo pot anchor points
  static final double POT_UPPER_LEN = 18.283; // inch
  static final double POT_LOWER_LEN = 18.18;  // inch
  static final double POT_OFFSET_E = 15.67;   //degres - off set from pot angle base

  // Strap / motor lengths - includes pully
  static final double STRAP_UPPER_LEN = 23.183;  // inch - includes pully takeup
  static final double STRAP_LOWER_LEN = 20.22;   // inch
  static final double STRAP_OFFSET_E = 19.99;    //degres - off set from pot angle base

  // Strap and yoyo pot have different angular offset based on the geometry
  static final double STRAP_OFFSET_ANGLE =  MAG_ANGLE_OFFSET - STRAP_OFFSET_E;
  static final double POT_OFFSET_ANGLE = MAG_ANGLE_OFFSET - POT_OFFSET_E;

  //law of cosind for yo-yo pot lengths
  final double POT_LENGTH_MIN = lawOfCosineLength(POT_LOWER_LEN, POT_UPPER_LEN, MIN_ANGLE - POT_OFFSET_ANGLE);
  final double POT_LENGTH_MAX = lawOfCosineLength(POT_LOWER_LEN, POT_UPPER_LEN, MAX_ANGLE - POT_OFFSET_ANGLE);

  //law of cosine constants for motor strap calcs
  static final double STRAP_LENGTH_MIN = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, MIN_ANGLE - STRAP_OFFSET_ANGLE);
  static final double STRAP_LENGTH_MAX = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, MAX_ANGLE - STRAP_OFFSET_ANGLE);
      
  /**
   * MagazinePositioner_Subsystem
   * 
   * By exposing the positioner as a subsystem it can be
   * be given it's own commands.
   * 
   */
  public class MagazinePositioner extends SubsystemBase {
    // sparkmax config
    final boolean kInverted = false;
    final int kPosSlot = 0;
    final int kVelSlot = 1;

    // we use velocity and positon modes, so two sets of PIDF values
    //pid values for motor P, I, D, F 
    PIDFController posPIDvalues = new PIDFController(0.1, 0.001, 0.1, 0.0);
    PIDFController velPIDvalues = new PIDFController(0.00002, 1e-7, 0.0, 0.0021);
    {
       posPIDvalues.setIzone(5.0); 
       velPIDvalues.setIzone(0.0);
    }
  
    // the FFHoldVolts might be useful to balance the shocks
    private double kArbFFHoldVolts = 0.0;

    // motor constants and scale factor
    static final double kGearRatio = 48.0;      // motor turns to pully turns
    static final double kPullyDiameter = 1.4;   // inches (1in without belting)
    static final double kInchPerMotorRev = Math.PI*kPullyDiameter / kGearRatio;
    static final double kRevPerInch = 1.0 / kInchPerMotorRev;
    static final double kMinVelZeroTol = 0.05;   //inches/sec
    static final double kMaxRPM = 30;

    // Pot Volts measured at top & bottom position
    static final double VatMin = 0.650; // volts at 22 degrees (min mag angle)
    static final double VatMax = 3.7438; // volts at 55 degrees (max mag angle)

    static final double kToleranceDeg = 0.2;
    
    // track encoder postion min/max - debugging
    double  min_encoder_pos;
    double  max_encoder_pos;

    // scale factors for yo-yo pot
    final double kInchPerVolt = (POT_LENGTH_MAX - POT_LENGTH_MIN) / (VatMax - VatMin);
    final double kDegPerVolt = ((MAX_ANGLE - MIN_ANGLE)/(VatMax - VatMin));

    // postion control devices
    final CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
    final CANEncoder angleEncoder = angleMotor.getEncoder();
    final CANPIDController anglePID = angleMotor.getPIDController();
    final AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);
    final DoubleSolenoid solenoid = new DoubleSolenoid(CAN.PCM2, PCM2.MAG_LOCK, PCM2.MAG_UNLOCK);
  
    // measurements
    double m_angle_linear = 0.0;    // mag angle linear estimate from pot
    double m_length_pot = 0.0;      // used to check measurements
    double m_length_pot_prev = 0.0; // used to measure speed
    double m_speed_pot = 0.0;       // dl/dt based on pot <unfiltered>
    double m_length_motor = 0.0;    // used to check measurements 
    double m_angle_pot = 0.0;       // mag angle using cosine and pot length
    double m_angle_motor = 0.0;     // mag angle using cosin and motor position
    double m_strap_speed = 0.0;     // inches/s
    double m_enc_pos = 0.0;         // raw 
    double m_strap_zero = 0.0;      // zero point for calibration
    boolean m_loose_strap = false;  // based on pot and motor speeds
    
    // setpoint values, used in periodic()
    double m_lengthSetpoint; // inches  <calculated from angleSetpoint>
    double m_angleSetpoint;  // degrees <input>

    MagazinePositioner() {
      SendableRegistry.setName(anglePot, this.getName(), "Mag Angle");
      
      // configure sparkmax motor
      angleMotor.restoreFactoryDefaults(false);
      angleMotor.setInverted(kInverted);
      angleMotor.setIdleMode(IdleMode.kBrake);
     
      //copy the sw pidvalues to the hardware
      posPIDvalues.copyTo(anglePID, kPosSlot);
      velPIDvalues.copyTo(anglePID, kVelSlot);
      angleMotor.burnFlash();
      angleEncoder.setPosition(0.0);

      //call periodic to read our values and calibrate
      periodic();
      calibrate();      // set motor encoder to zero and star
      zeroPower(true);  // relase any holding, lock gear so mag stays in place
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
      layout.addNumber("MAGPos/angle_lin", () -> m_angle_linear).withSize(2, 5);
      layout.addNumber("MAGPos/angle_pot", () -> m_angle_pot); 
      layout.addNumber("MAGPos/angle_mot", () -> m_angle_motor); 
      layout.addNumber("MAGPos/len_pot",   () -> m_length_pot);
      layout.addNumber("MAGPos/len_strap", () -> m_length_motor );
      layout.addNumber("MAGPos/encoder",   () -> m_enc_pos);
    }


    @Override
    public void periodic() {
      m_length_pot_prev = m_length_pot;

      // read pot to get length of pot, then calculate the angle based on potentiometer 
      double apv =anglePot.getAverageVoltage();
      m_length_pot =  (apv - VatMin) * kInchPerVolt + POT_LENGTH_MIN;
      m_angle_pot = lawOfCosineAngle(POT_LOWER_LEN, POT_UPPER_LEN, m_length_pot) + POT_OFFSET_ANGLE;
      
      // use linear estimate should be good enough based on spreadsheet 
      m_angle_linear = (apv - VatMin) * kDegPerVolt + MIN_ANGLE;
      
      // nw calc the angle based on the motor length
      m_enc_pos =  angleEncoder.getPosition();
      m_length_motor = m_strap_zero + m_enc_pos* kInchPerMotorRev;
      m_angle_motor = lawOfCosineAngle(STRAP_LOWER_LEN, STRAP_UPPER_LEN, m_length_motor) + STRAP_OFFSET_ANGLE;
    
      //measure strap speed
      m_strap_speed = angleEncoder.getVelocity()*(kInchPerMotorRev/60.0);

      // estimate pot speed inch/s
      m_speed_pot = (m_length_pot_prev - m_length_pot) / DT;
      isMoving();
      safety();
    }

    /**
     * calibrate() - uses the pot_length and sets the motor_stap zero position
     *  to the current point.  The strap must not have slack for this to work as 
     *  expected. If the motor angle estimate and the pot angle estimate differ
     *  by too much, this function should get called again.
     *   
     *  If that is happening, the geometry measurements are off somewhere.
     * 
     * State variable used:
     *    m_pot_lenght     <input>
     *    m_strap_zero     <output>
     *    m_angleSetpoint  <output>
     * 
     */
    public void calibrate() {
      // use POT angle to calculate strap length
      double potang = lawOfCosineAngle(POT_LOWER_LEN, POT_UPPER_LEN, m_length_pot);
      double strapang = potang + (POT_OFFSET_ANGLE - STRAP_OFFSET_ANGLE);  //
      
      // calculate the strap length and use it as the zero point on the motor encoder
      m_strap_zero = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, strapang);
      angleEncoder.setPosition(0.0); 

      // calculate min/bax postion rotations - motor units
      min_encoder_pos = (STRAP_LENGTH_MIN - m_strap_zero) * kRevPerInch;
      max_encoder_pos = (STRAP_LENGTH_MAX - m_strap_zero) * kRevPerInch;

      // force angle setpoint to current position
      m_angleSetpoint = potang + POT_OFFSET_ANGLE;
    }

    /**
     * gets Magazine angle in robot coordinates
     * @return
     */
    public double get() {
      return m_angle_motor;
    }

    public void setAngle(double magDeg) {
      //limit range and save our current setpoint
      m_angleSetpoint = MathUtil.limit(magDeg, MIN_ANGLE, MAX_ANGLE);

      //figure out desired motor strap length
      double strap_deg = m_angleSetpoint - STRAP_OFFSET_ANGLE;
      m_lengthSetpoint = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, strap_deg);

      //calculate motor postion in revs from our calibration positon
      double setpoint = (m_lengthSetpoint - m_strap_zero)*kRevPerInch;
      
      // unlock the gear before moving, tell angleMotor to go to position
      unlock();
      CANError err = anglePID.setReference(setpoint, ControlType.kPosition, kPosSlot);
      if (err != CANError.kOk) {
        System.out.println("SMARTMAX CAN ERROR in MAG POSITION"+ err);
      }
    }

    /**
     * use velocity control to wind motor
     * @param speed  RPM of takeup pully
     */
    public void wind(double pully_rpm) {
      if ((isAtBottom() && pully_rpm < 0.0) || (isAtTop() && pully_rpm > 0.0)) {
        pully_rpm = 0.0;
      }

      pully_rpm = MathUtil.limit(pully_rpm, -kMaxRPM, kMaxRPM);
      double motor_speed = kGearRatio*pully_rpm;
      unlock();
      anglePID.setReference(motor_speed, ControlType.kVelocity, kVelSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage); 
    }

    /**
     * zeroPower()  - sets motor speed to 0.0
     *  stops using position or velocity mode control 
     *  puts motor in idle mode
     *  optionally lock the holding gear
     * 
     *  This will let the struts push the magazine up if lock is false
     */
    public void zeroPower(boolean lock) {
      angleMotor.set(0.0);
      if (lock) lock();
    }

    /**
     * stopAndHold()  - use position control to hold the motor
     * @param lock  true, engage lock
     *              false - leave it open
     */
    public void stopAndHold(boolean lock) {
      // angleMotor.stopMotor(); //zero's motor current, idle mode hold only
      setAngle(get());       // sets postion, keeps motor in position control, uses current position
      if (lock) lock();
    }

    public boolean isAtBottom() {
      return (get() <= MIN_SOFT_STOP)  ?  true : false;
    }
    
    public boolean isAtTop() {
      return (get() >= MAX_SOFT_STOP)  ?  true : false;
    }
    
    public boolean isAtSetpoint() {
      return (Math.abs(get() - m_angleSetpoint) < kToleranceDeg)  ? true : false;     
    }

    //checks to make sure we don't do stupid
    void safety() {
      //not sure what stupid is yet
      //maybe a current check for being against the stop?

      // if we get outside the range, recalibrate against the pot.  
      if (Math.abs(m_angle_motor - m_angle_pot) > 2.0) {
        System.out.println("Mag Angle Calibration Event ang_mot=" + m_angle_motor + " ang_pot=" + m_angle_pot );
        calibrate();
      }  
    }
  

    /**
     * controls for the lock on the 
     */
    public void lock() {
      solenoid.set(Value.kForward);
    }
  
    public void unlock() {
      solenoid.set(Value.kReverse);
    }
  
    public boolean isLocked() {
      return (solenoid.get() == Value.kForward);
    }

    public boolean isMoving() {
      // possible motor is moving and mag isn't 
      boolean mag_moving = Math.abs(m_speed_pot) > kMinVelZeroTol;
      boolean motor_moving = Math.abs(m_strap_speed) > kMinVelZeroTol;
      m_loose_strap = (mag_moving != motor_moving);

      return (mag_moving || motor_moving);
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
  public void periodic() { }

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
    positioner.setAngle(MAX_SOFT_STOP);
  }

  public void lower() {
    positioner.setAngle(MIN_SOFT_STOP);
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

  /**
   *  Mostly for access in tests.
   * 
   * @return positioner
   */
  public MagazinePositioner getMagPositioner() {
    return positioner;
  }

  /**
   * 
   * @param b length
   * @param c length
   * @param a_deg
   * @return   length for side a, given A, b, c
   */
  static double lawOfCosineLength( double b, double c, double a_deg) {
    return Math.sqrt(b*b + c*c - 2.0*b*c * Math.cos(Math.toRadians(a_deg)));
  }

  /**
   *
   * Note this could be sped up by pre-calc of constants in the expression.
   *  
   * @param a length
   * @param b length
   * @param c length
   * @return   angle of C, opposite side c
   */
  static double lawOfCosineAngle(double a, double b, double c) {
     double cos = (a*a + b*b - c*c) / (2*a*b);
     double acos = Math.toDegrees(Math.acos(cos));
     return acos;
  }

}
