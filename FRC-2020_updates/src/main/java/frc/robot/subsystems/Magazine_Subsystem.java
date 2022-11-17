// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.MathUtil.clamp;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants.AnalogIn;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PCM2;
import frc.robot.Constants.PWM;
import frc.robot.subsystems.util.MonitoredSubsystemBase;
import frc.robot.util.misc.PIDFController;

public class Magazine_Subsystem extends MonitoredSubsystemBase {
  // Physical limits
  public static final double MIN_ANGLE = 20.0;  //measured at mechanical stops
  public static final double MAX_ANGLE = 59.5;  //measured at mechanical limit

  // Pot Volts measured at top & bottom position <measured>
  static final double VatMin = 0.1770; // volts at 20.7 degrees (min mag angle)
  static final double VatMax = 4.3225;  // volts at 59.5 degrees (max mag angle)

  public static final double MAX_SOFT_STOP = 55.5;
  public static final double MIN_SOFT_STOP = 22.0;

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
  static final double MAG_ANGLE_OFFSET = 28.10; //degrees - rotation due to frame

  // arm lengths for yo-yo pot anchor points
  static final double POT_UPPER_LEN = 16.625; // inch - c"
  static final double POT_LOWER_LEN = 15.875;  // inch - d"
  static final double POT_OFFSET_E = 14.13;   //degrees - offset from pot angle base -E"

  // Strap / motor lengths - includes pully
  static final double STRAP_UPPER_LEN = 23.12;  // inch, includes pully (unwound) - c'
  static final double STRAP_LOWER_LEN = 20.58;   // inch - d'
  static final double STRAP_OFFSET_E = 22.60;    //degres - offset from pot angle base E

  // Strap and yoyo pot have different angular offset based on the geometry
  static final double STRAP_OFFSET_ANGLE =  MAG_ANGLE_OFFSET - STRAP_OFFSET_E;
  static final double POT_OFFSET_ANGLE = MAG_ANGLE_OFFSET - POT_OFFSET_E;

  //law of cosind for yo-yo pot lengths
  final double POT_LENGTH_MIN = lawOfCosineLength(POT_LOWER_LEN, POT_UPPER_LEN, MIN_ANGLE - POT_OFFSET_ANGLE);
  final double POT_LENGTH_MAX = lawOfCosineLength(POT_LOWER_LEN, POT_UPPER_LEN, MAX_ANGLE - POT_OFFSET_ANGLE);

  //law of cosine constants for motor strap calcs
  static final double STRAP_LENGTH_MIN = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, MIN_ANGLE - STRAP_OFFSET_ANGLE);
  static final double STRAP_LENGTH_MAX = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, MAX_ANGLE - STRAP_OFFSET_ANGLE);
  
  private NetworkTable table;
  private NetworkTableEntry nt_angle;
  private NetworkTableEntry nt_pcCount;
  private NetworkTableEntry nt_ballOne;
  private NetworkTableEntry nt_ballTwo;
  private NetworkTableEntry nt_ballThree;
  private int NT_update = 0;

  /**
   * MagazinePositioner_Subsystem
   * 
   * By exposing the positioner as a subsystem it can be be given it's own
   * commands.
   * 
   */
  public class MagazinePositioner extends MonitoredSubsystemBase {
    // sparkmax config
    final boolean kInverted = true;
    final int kPosSlot = 0;
    final int kVelSlot = 1;

    // we use velocity and positon modes, so two sets of PIDF values
    // pid values for motor P, I, D, F
    PIDFController posPIDvalues = new PIDFController(0.25, 0.001, 5, 0.0);
    PIDFController velPIDvalues = new PIDFController(0.000028, 1e-7, 0.0, 0.0035);
    {
      posPIDvalues.setIzone(3.0);
      velPIDvalues.setIzone(0.0);
    }

    // the FFHoldVolts might be useful to balance the shocks
    private double kArbFFHoldVolts = 0.0;

    // motor constants and scale factor
    static final double kGearRatio = 48.0; // motor turns to pully turns
    static final double kPullyDiameter = 1.4; // inches (1in without belting)
    static final double kInchPerMotorRev = Math.PI * kPullyDiameter / kGearRatio;
    static final double kRevPerInch = 1.0 / kInchPerMotorRev;
    static final double kMinVelZeroTol = 0.05; // inches/sec
    static final double kMaxRPM = 30;

    static final double kToleranceDeg = 0.25;

    // track encoder postion min/max - debugging
    double min_encoder_pos;
    double max_encoder_pos;

    // scale factors for yo-yo pot
    final double kInchPerVolt = (POT_LENGTH_MAX - POT_LENGTH_MIN) / (VatMax - VatMin);
    final double kDegPerVolt = ((MAX_ANGLE - MIN_ANGLE) / (VatMax - VatMin));
    
    //Calculate a safety pot voltage that should never be under if soft stop is maintained
    final double VSafety = (MIN_SOFT_STOP - 1.0 - MIN_ANGLE)/kDegPerVolt + VatMin;

    // postion control devices
    final CANSparkMax angleMotor = new CANSparkMax(CAN.MAG_SMAX, MotorType.kBrushless);
    final RelativeEncoder angleEncoder = angleMotor.getEncoder();
    final SparkMaxPIDController anglePID = angleMotor.getPIDController();
    final AnalogInput anglePot = new AnalogInput(AnalogIn.MAGAZINE_ANGLE);
    final DoubleSolenoid solenoid = new DoubleSolenoid(CAN.PCM2, PCM2.MAG_LOCK, PCM2.MAG_UNLOCK);

    // measurements updated in periodic 
    double m_apv = 0;            // angle pot avg voltage, safety()
    double m_enc_pos = 0.0;      // encoder position 
    double m_length_motor = 0.0; // 
    double m_angle_motor = 0.0;  // mag angle, from encoder pos
    double m_strap_speed = 0.0;  // inches/s
    
    // set during calibration
    double cal_strap_zero = 0.0; // zero encoder strap length
    double cal_length_pot = 0.0; // used to check measurements
    double cal_angle_pot = 0.0;  // mag angle measured using cosine and pot length

    double m_pully_rpm = 0;

    // setpoint values
    double m_angle_setpoint;    // degrees <input>
    double m_length_setpoint;   // inches  <calculated from angleSetpoint>
    double m_enc_setpoint;      // encoder <calculated count postion>
    
    // tracking the pawl, do bet to protect the neo motor
    boolean m_unlock_confirmed; // true down-motion detected so pawl should be free

    // network tables
    private NetworkTable table;
    private NetworkTableEntry nt_angle_mot;
    private NetworkTableEntry nt_strap_speed;
    private NetworkTableEntry nt_len_strap;
    private NetworkTableEntry nt_encoder;
    private NetworkTableEntry nt_encoder_sp;
    private NetworkTableEntry nt_angle_pot_calib;


    /**
     * 
     * MagazinePostioner 
     * 
     * Controls the angle of the magazine using winch motor working 
     * against spring lifts.  A pawl is used to lock the position
     * for power off to prevent unwinding or when you want to shoot 
     * from a fixed angle.
     * 
     * Moving down, the pawl will "click" moving up requires the pawl to be
     * unloaded (down move) and the released.
     * 
     * A linear -yo-yo pot is used to calibrate the initial position, then
     * the angle is controled by the SparkMax position control.
     * 
     */

    MagazinePositioner() {
      SendableRegistry.setName(anglePot, this.getName(), "YoYoPot");

      // configure sparkmax controller
      angleMotor.restoreFactoryDefaults(false);
      angleMotor.setInverted(kInverted);
      angleMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setSmartCurrentLimit(5);        // stall current limit

      // copy the sw pidvalues to the hardware
      posPIDvalues.copyTo(anglePID, kPosSlot);
      velPIDvalues.copyTo(anglePID, kVelSlot);
      angleMotor.burnFlash();

      // network tables setup
      table = NetworkTableInstance.getDefault().getTable("MAGPos");
      nt_angle_mot = table.getEntry("AngleMot");
      nt_strap_speed = table.getEntry("StrapSpeed");
      nt_angle_pot_calib = table.getEntry("calib_angle_pot");
      nt_len_strap = table.getEntry("LenStrap");
      nt_encoder = table.getEntry("Encoder");
      nt_encoder_sp = table.getEntry("EncoderSP");

      // set motor encoder to zero and calculate 
      calibrate(); 
      periodic();
      zeroPower(false); 
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
      // layout.addNumber("MAGPos/angle_mot", () -> m_angle_motor);
      // layout.addNumber("MAGPos/strap_speed", () -> m_strap_speed);
    }

    public void addDebugDashboardWidgets(ShuffleboardLayout layout) {
      /*
       * layout.addNumber("MAGPos/angle_pot", () -> cal_angle_pot).withSize(2, 5);;
       * layout.addNumber("MAGPos/angle_mot", () -> m_angle_motor);
       * layout.addNumber("MAGPos/len_pot", () -> cal_length_pot);
       * layout.addNumber("MAGPos/len_strap", () -> m_length_motor );
       * layout.addNumber("MAGPos/encoder", () -> m_enc_pos);
       * layout.addNumber("MAGPos/encoder_sp", () -> m_enc_setpoint);
       */
    }

    @Override
    public void monitored_periodic() {
      // update pot voltage for monitoring and safety()
      m_apv = anglePot.getAverageVoltage();

      // Calculate the angle based on the motor strap length
      m_enc_pos = angleEncoder.getPosition();
      m_length_motor = cal_strap_zero + m_enc_pos * kInchPerMotorRev;
      m_angle_motor = lawOfCosineAngle(STRAP_LOWER_LEN, STRAP_UPPER_LEN, m_length_motor) + STRAP_OFFSET_ANGLE;

      // measure strap speed [in/s] of the strap
      m_strap_speed = angleEncoder.getVelocity() * (kInchPerMotorRev / 60.0);

      updateNetworkTables();
      safety();
    }

    void updateNetworkTables() {
      // network table outputs
      if ((NT_update++ % 10) == 0) { // every 10 cycles since no log extension
        nt_angle_mot.setDouble(m_angle_motor);
        nt_strap_speed.setDouble(m_strap_speed);
        nt_len_strap.setDouble(m_length_motor);
        nt_encoder.setDouble(m_enc_pos);
      }
    }

    /**
     * calibrate() - uses the pot_length and sets the motor_stap zero position to
     * the current point. The strap must not have slack for this to work as
     * expected. If the motor angle estimate and the pot angle estimate differ by
     * too much, this function should get called again.
     * 
     * If that is happening, the geometry measurements are off somewhere.
     * 
     * State variable used: cal_angle_pot <input> calibrate to measured pot lenght
     * cal_strap_zero <output> m_angle_setpoint <output>
     * 
     */
    public void calibrate() {
      // read pot to get length of pot, then calculate the angle based on
      // potentiometer
      m_apv = anglePot.getAverageVoltage();
      cal_length_pot = (m_apv - VatMin) * kInchPerVolt + POT_LENGTH_MIN;
      cal_angle_pot = lawOfCosineAngle(POT_LOWER_LEN, POT_UPPER_LEN, cal_length_pot) + POT_OFFSET_ANGLE;

      // use POT angle to calculate strap length
      m_angle_setpoint = cal_angle_pot; // measured in periodic()

      // figure out the current strap length and angle, this position will be our zero
      double strap_deg = m_angle_setpoint - STRAP_OFFSET_ANGLE;
      cal_strap_zero = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, strap_deg);
      m_length_setpoint = cal_strap_zero;

      // calculate motor postion in revs from our calibration positon
      m_enc_setpoint = 0.0; // (m_length_setpoint - cal_strap_zero)*kRevPerInch

      // calculate min/max postion rotations - motor units
      min_encoder_pos = (STRAP_LENGTH_MIN - cal_strap_zero) * kRevPerInch;
      max_encoder_pos = (STRAP_LENGTH_MAX - cal_strap_zero) * kRevPerInch;

      // set current pos as zero, use the limits as soft limits on the motor
      // controller
      angleEncoder.setPosition(0.0);
      angleMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) min_encoder_pos);
      angleMotor.setSoftLimit(SoftLimitDirection.kForward, (float) max_encoder_pos);
      
      // set our reference to zero as well
      anglePID.setReference(m_enc_setpoint, ControlType.kPosition, kPosSlot);
      
      // update nt
      nt_angle_pot_calib.setDouble(cal_angle_pot);
      nt_encoder_sp.setDouble(m_enc_setpoint);
    }

    /**
     * gets Magazine angle in robot coordinates
     * 
     * @return
     */
    public double get() {
      return m_angle_motor;
    }

    public void setAngle(double magDeg) {
      // limit range and save our current setpoint
      m_angle_setpoint = clamp(magDeg, MIN_SOFT_STOP, MAX_SOFT_STOP);

      // figure out desired motor strap length
      double strap_deg = m_angle_setpoint - STRAP_OFFSET_ANGLE;
      m_length_setpoint = lawOfCosineLength(STRAP_LOWER_LEN, STRAP_UPPER_LEN, strap_deg);

      // calculate motor postion in revs from our calibration positon
      m_enc_setpoint = (m_length_setpoint - cal_strap_zero) * kRevPerInch;

      // we need to give the pawl a chance to unlock, so don't hit hardware if unsafe
      if (getUnlockConfirmed() == false)
        return;

      // issue the command
      anglePID.setReference(m_enc_setpoint, ControlType.kPosition, kPosSlot);
      nt_encoder_sp.setDouble(m_enc_setpoint);
    }

    /**
     * Uses velocity control to wind motor, +speed increases angle, -speed decreases
     * angle.
     * 
     * @param speed RPM of takeup pully
     */
    public void wind(double pully_rpm) {
      if ((isAtBottom() && pully_rpm < 0.0) || (isAtTop() && pully_rpm > 0.0)) {
        return;
      }

      // guard against pulling into pawl
      if ((pully_rpm > 0) && isLocked() && !m_unlock_confirmed) {
        // we have to burp the motor and make sure we are unlocked
        unlock(); // can't trust this one if there is any tension
        return;
      }
      unlock();
      m_pully_rpm = clamp(pully_rpm, -kMaxRPM, kMaxRPM);
      double motor_speed = kGearRatio * m_pully_rpm;
      anglePID.setReference(motor_speed, ControlType.kVelocity, kVelSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
    }

    /**
     * burp() - offload the pawl by moving in down direction. This must be
     * controlled by a Command so movement logic can be done over multiple frames.
     */
    public void burp() {
      final double inps = 0.3;
      double burpSpeed = -(kRevPerInch * 60) * inps; // [rev/in][sec/min][in/sec] = [rpm] Down

      // burp the motor to unlock the pawl, i.e. go back slowly
      anglePID.setReference(burpSpeed, ControlType.kVelocity, kVelSlot, kArbFFHoldVolts, ArbFFUnits.kVoltage);
    }

    /**
     * zeroPower() - sets motor speed to 0.0 stops using position or velocity mode
     * control puts motor in idle mode optionally lock the holding gear
     * 
     * This will let the struts push the magazine up if lock is false
     */
    public void zeroPower(boolean lock) {
      angleMotor.stopMotor();
      if (lock)
        lock();
    }

    /**
     * stopAndHold() - use position control to hold the motor
     * 
     * @param lock true, engage lock false - leave it open
     */
    public void stopAndHold(boolean lock) {
      m_enc_pos = angleEncoder.getPosition();
      anglePID.setReference(m_enc_pos, ControlType.kPosition, kPosSlot);
      if (lock)
        lock();
    }

    public boolean isAtBottom() {
      return (get() <= MIN_SOFT_STOP) ? true : false;
    }

    public boolean isAtTop() {
      return (get() >= MAX_SOFT_STOP) ? true : false;
    }

    public boolean isAtSetpoint(double angle_setpoint) {
      return (Math.abs(get() - angle_setpoint) < kToleranceDeg) ? true : false;
    }

    // checks to make sure we don't do stupid
    void safety() {
      // monitor analog pot getting too close to limit. Just kill it
      if (m_unlock_confirmed == false && m_apv < VSafety) {
        zeroPower(false);
        if (!RobotBase.isSimulation()) {  //supress this warning if in simulation mode
          DriverStation.reportWarning("Mag Angle too close to limit during burp - shutting down. Check pawl.", false);
        }
      }
    }

    /**
     * called by a Command when down motion is detected
     */
    public void unlockConfirmed() {
      m_unlock_confirmed = true;
    }

    public boolean getUnlockConfirmed() {
      return m_unlock_confirmed;
    }

    /**
     * controls for the lock on the
     */
    public void lock() {
      solenoid.set(Value.kForward);
      m_unlock_confirmed = false;
    }

    public void unlock() {
      solenoid.set(Value.kReverse);
    }

    public boolean isLocked() {
      return (solenoid.get() == Value.kForward);
    }

    public boolean isMoving() {
      m_strap_speed = angleEncoder.getVelocity() * (kInchPerMotorRev / 60.0);
      // possible motor is moving and mag isn't
      boolean motor_moving = Math.abs(m_strap_speed) > kMinVelZeroTol;
      return motor_moving;
    }

    /**
     * isMovingDown means we could be releasing load on the pawl
     */
    public boolean isMovingDown() {
      m_strap_speed = angleEncoder.getVelocity() * (kInchPerMotorRev / 60.0);
      // possible motor is moving and mag isn't
      boolean motor_moving = m_strap_speed < -kMinVelZeroTol;
      return motor_moving;
    }

  } // MagazinePositioner


  
  

  // magazine Belt and gate devices
  final Spark beltMotor = new Spark(PWM.MAGAZINE);
  final DigitalInput lightGate = new DigitalInput(DigitalIO.MAGAZINE_GATE);
  
  // magazine angle positioner
  final MagazinePositioner positioner;

  // measurements update in periodic()
  int m_pcCount = 0;
  int NT_update_mag = 0;
  
  final int MAG_FULL_COUNT = 3;   //number of power cells
  
  /** Creates a new Magazine_subsystem. 
   * 
   * Magazine Belt and Angle controls
   * 
   * Controls the mag belt and power cell counting using a light gate.
   * Most of the work is done in the MagazineCaptureCmd which runs as a default
   * command for loading power cells. 
   *
   * The postioner moves the whole magazine up and down. Commands may take over the
   * positioner sub-system or the belt
   *
   * 
  */
  public Magazine_Subsystem() {
    this.positioner = new MagazinePositioner();

    //direct networktables logging
    table = NetworkTableInstance.getDefault().getTable("Magazine");
    nt_angle= table.getEntry("MagazineAngle");
    nt_pcCount = table.getEntry("PCCount");
    nt_ballOne = table.getEntry("BallOne/value");
    nt_ballTwo = table.getEntry("BallTwo/value");
    nt_ballThree = table.getEntry("BallThree/value");

    pcUpdateNetworkTables();

    // fill out dashboard stuff - commented out to save bandwidth
    //SendableRegistry.setSubsystem(this, "Magazine");
    //SendableRegistry.setName(lightGate, this.getName(), "Mag LightGate");
    //SendableRegistry.setName(beltMotor, this.getName(), "Mag Belt");
  }

  @Override
  public void monitored_periodic() {
        updateNetworkTablesMag();
   }

   void updateNetworkTablesMag() {
    // network table outputs every 10 cycles
    if ((NT_update_mag++ % 10) == 0){
      nt_angle.setNumber(positioner.get());
    }
  }


  public void beltOn(double motorStrength) {
    beltMotor.set(motorStrength);
  }

  public void beltOff() {
    beltMotor.set(0);
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

  //for webpage magic, need three separate booleans for the balls for some cool graphics
  public void pcUpdateNetworkTables() { 
    nt_pcCount.setNumber(m_pcCount);
    nt_ballOne.setBoolean((m_pcCount & 1) !=0);
    nt_ballTwo.setBoolean((m_pcCount & 2) !=0);
    nt_ballThree.setBoolean((m_pcCount & 3) !=0);
  }

  public void addPC() {
    m_pcCount++;
    pcUpdateNetworkTables();
  }

  public void removePC() {
    if (m_pcCount > 0)
      m_pcCount--;
      pcUpdateNetworkTables();
  }

  public int getPC() {
    return m_pcCount;
  }

  public void setPC(int c) {
    m_pcCount = (c >= 0  && c <= MAG_FULL_COUNT) ? c : 0;
    pcUpdateNetworkTables();
  }

  public boolean isMagFull() {
    return (m_pcCount >= MAG_FULL_COUNT);
  }

  public boolean isMagEmpty() {
    return (m_pcCount == 0);
  }

  public double getAngle() {
    return positioner.get();
  }

  /**
   *  Access for tests and commands
   * 
   * @return positioner
   */
  public MagazinePositioner getMagPositioner() {
    return positioner;
  }

  /**
   * calculate length of side given two lengths and opposite angle
   * 
   * @param b length
   * @param c length
   * @param a_deg
   * @return  length for side a, given A, b, c
   */
  static double lawOfCosineLength( double b, double c, double a_deg) {
    return Math.sqrt(b*b + c*c - 2.0*b*c * Math.cos(Math.toRadians(a_deg)));
  }

  /**
   *  calculate angle given lengths of triangle.
   *  
   * @param a length
   * @param b length
   * @param c length
   * @return  angle of C, opposite side c
   */
  static double lawOfCosineAngle(double a, double b, double c) {
     double cos = (a*a + b*b - c*c) / (2*a*b);
     double acos = Math.toDegrees(Math.acos(cos));
     return acos;
  }

}
