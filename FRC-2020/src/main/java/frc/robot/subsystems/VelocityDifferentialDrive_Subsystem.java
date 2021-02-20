package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RobotPhysical.WheelAxleDistance;
import static frc.robot.Constants.RobotPhysical.WheelDiameter;
import static frc.robot.Constants.RobotPhysical.WheelWearLeft;
import static frc.robot.Constants.RobotPhysical.WheelWearRight;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.RamseteProfile;
import frc.robot.Constants.RobotPhysical;
import frc.robot.RobotContainer;
import frc.robot.sim.AHRS_GyroSim;
import frc.robot.sim.EncoderSim2;
import frc.robot.subsystems.ifx.DashboardUpdate;
import frc.robot.subsystems.ifx.DualDrive;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.Shifter.Gear;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.subsystems.ifx.VoltageDrive;
import frc.robot.util.misc.MathUtil;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase
    implements Logger, DualDrive, VelocityDrive, VoltageDrive, DashboardUpdate {
  // Sign conventions, apply to encoder and command inputs
  // Brushless SparkMax cannot invert the encoders
  final double Kleft = 1.0;
  final double Kright = -1.0;
  final boolean KInvertMotor = true; // convention required in robot characterization
  final IdleMode KIdleMode = IdleMode.kCoast;
  final double Kgyro = -1.0; // ccw is positive, just like geometry class

  private NetworkTable table;
  private NetworkTableEntry nt_velLeft;
  private NetworkTableEntry nt_velRight;
  private NetworkTableEntry nt_posLeft;
  private NetworkTableEntry nt_posRight;
  private NetworkTableEntry nt_theta;
  private NetworkTableEntry nt_accelX;
  private NetworkTableEntry nt_accelY;
  private NetworkTableEntry nt_accelZ;
  
  // we only use this one PID slot for the drive lead controllers
  final int KpidSlot = 0;

  // Current Limits
  private int smartCurrentLimit = DriveTrain.smartCurrentLimit; // amps
  // private final double KSecondaryCurrent = 1.40; // set secondary current based
  // on smart current

  // Phyical units deadzone
  private final double RPM_DZ = 2.0;

  // Acceleration limits
  private double slewRateLimit = DriveTrain.slewRateLimit; // seconds to max speed/power

  // Chasis details
  private final double K_ft_per_rev = Math.PI * WheelDiameter / 12.0; // feet/rev
  private final double K_low_fps_rpm; // Low gear ft/s / rpm of motor shaft
  private final double K_high_fps_rpm; // High gear ft/s /rpm of motor shaft

  // CANSpark Max will be used 3 per side, 2 folowing the leader
  private final CANSparkMax frontRight = new CANSparkMax(CAN.FR_SMAX, kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(CAN.FL_SMAX, kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(CAN.BR_SMAX, kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(CAN.BL_SMAX, kBrushless);
  private final CANSparkMax middleRight = new CANSparkMax(CAN.MR_SMAX, kBrushless);
  private final CANSparkMax middleLeft = new CANSparkMax(CAN.ML_SMAX, kBrushless);

  private final CANSparkMax[] controllers = new CANSparkMax[] {
     frontRight, frontLeft, 
     backRight,  backLeft, 
     middleRight, middleLeft };

  // split left/right sides controller/encoder/pid
  final CANSparkMax leftController = backLeft;
  final CANSparkMax rightController = backRight;
  final CANEncoder leftEncoder = leftController.getEncoder();
  final CANEncoder rightEncoder = rightController.getEncoder();
  final CANPIDController leftPID = leftController.getPIDController();
  final CANPIDController rightPID = rightController.getPIDController();

  // Voltage to get robot to move.
  // kS - taken from the Drive Characterization
  // - for reference, 0.15 was enough to move 2019 arm-bot chassis
  private final double ARBIT_FEEDFWD_MAX_VOLT = 1.5; // volts max allowed
  private double arbFeedFwd = RamseteProfile.ksVolts; // current value

  // Calculated based on desired low-gear max ft/s - UX may update 
  double maxFPS_High; // <input>
  double maxFPS_Low; // using HIGH gear max RPM
  double maxRPM_High; // max motor RPM low & high
  double maxDPS; // max rotation in deg/sec around Z axis

  // drivetrain & gear objects
  final DifferentialDrive dDrive;
  Shifter gearbox;
  Gear requestedGear; // where driver or external logic wants the gear
  boolean coastMode = false; // attempt to coast when Vcmd > Vrequest

  // measurements, taken in periodic(), robot coordinates
  double m_velLeft = 0.0; // fps, positive moves robot forward
  double m_velRight = 0.0; // fps, positive moves robot forward
  double m_posLeft = 0.0; // feet, positive is forward distance, since encoder reset
  double m_posRight = 0.0; // feet, positive is forward distance, since encoder reset
  Gear m_currentGear; // high/low
  double m_theta; // heading
  double m_voltleft; // save voltages that we send to motor
  double m_voltright;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Simulation - these classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private EncoderSim2 m_leftEncoderSim;
  private EncoderSim2 m_rightEncoderSim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
  private AHRS_GyroSim m_gyroSim;

  public VelocityDifferentialDrive_Subsystem(final Shifter gear) {
    // save scaling factors, they are required to use SparkMax in Vel mode
    gearbox = gear;
    requestedGear = gearbox.getCurrentGear();


   //direct networktables logging
    table = NetworkTableInstance.getDefault().getTable("Drivetrain");
    nt_velLeft = table.getEntry("VelLeft");
    nt_velRight = table.getEntry("VelRight");
    nt_posLeft = table.getEntry("PosLeft");
    nt_theta = table.getEntry("Theta");
    nt_accelX = table.getEntry("x");
    nt_accelY = table.getEntry("y");
    nt_accelZ = table.getEntry("z");

    // setup physical units - chassis * gearbox (rev per minute)
    K_low_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.LOW) / 60; // rpm/60 rps
    K_high_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.HIGH) / 60;

    //Speed setting may be updated via UX, but set defaults
    calcSpeedSettings();

    // setup SparkMax controllers, sets left and right masters
    configureControllers();

    // create a dDrive to support other modes besides velocity command
    dDrive = new DifferentialDrive(leftController, rightController);
    dDrive.setSafetyEnabled(DriveTrain.safetyEnabled);

    // this may cause a 1-2 second delay - robot can't move during period
    m_gyro.enableBoardlevelYawReset(true);
    m_gyro.calibrate();
    while (m_gyro.isCalibrating()) { // wait to zero yaw if calibration is still running
      try {
        Thread.sleep(250);
        System.out.println("calibrating gyro");
      } catch (InterruptedException e) {
        /* do nothing */ }
    }
    System.out.println("Pre-Zero yaw:" + m_gyro.getYaw());
    m_gyro.reset(); // should zero yaw but not working.
    System.out.println("Post-Zero yaw:" + m_gyro.getYaw());

    // init simulation iff needed
    if (RobotBase.isSimulation())
      setupSimulation();

    // clear our starting position.
    resetPosition();
    m_odometry = new DifferentialDriveOdometry(readGyro());
  }

  void calcSpeedSettings() {
    // compute max RPM for motors for high and low gears
    var dp = RobotContainer.getInstance().getDriverPreferences();
    maxDPS = (dp == null) ? DriveTrain.maxRotDPS : dp.getMaxRotation();
    maxFPS_High = (dp == null) ? DriveTrain.maxFPS : dp.getMaxSpeed();
    maxRPM_High = (maxFPS_High / K_high_fps_rpm);
    maxFPS_Low = (DriveTrain.motorMaxRPM * K_low_fps_rpm);

    // don't allow Motor RPM to let low speed be faster than high speed
    // which could happen with a slow user choice.
    maxFPS_Low = (maxFPS_Low > maxFPS_High) ? maxFPS_High : maxFPS_Low;

    // check we can hit max requested speed in high gear
    if (maxRPM_High > DriveTrain.motorMaxRPM) {
      System.out.println("Warning: maxFPS not reachable. maxFPS= " + (DriveTrain.motorMaxRPM * K_high_fps_rpm));
    }
    if (maxFPS_Low >= maxFPS_High) {
      System.out.println("Warning: maxFPS low/high overlap" + maxFPS_Low + ">=" + maxRPM_High);
    }
  }

  // Process any Dashboard events
  public void processDashboard(EntryNotification event) {
    // don't worry about which event, grab them all
    calcSpeedSettings();
  }

  /**
   * hides some of the ugly setup for our collection of controllers
   */
  void configureControllers() {
    resetControllers();

    // Have motors follow the main left/right controller
    middleRight.follow(rightController);
    frontRight.follow(rightController);
    middleLeft.follow(leftController);
    frontLeft.follow(leftController);

    // configure lead controller's pid
    DriveTrain.pidValues.copyTo(leftPID, KpidSlot);
    DriveTrain.pidValues.copyTo(rightPID, KpidSlot);

    // zero adjust will set the default limits for accel and currents
    adjustAccelerationLimit(0.0);
    adjustFeedForward(0.0);
    adjustCurrentLimit(0);

    // burn the default value in case of brown-out
    saveControllers();
  }

  @Override
  public void periodic() {
    m_currentGear = gearbox.getCurrentGear();
    m_theta = Kgyro * m_gyro.getYaw();
    double kGR = gearbox.getGearRatio();
    
    /**
     * Encoder.getPostion() is in units of revs of motor shaft K_ft_per_rev -
     * pi*wheeldiam - distance per rev of wheel shaft
     * 
     * Wheelwear accounts for calibration, expect 0.9 < wear <= 1.0
     * K_ft_per_rev*WheelWear = actual distance traveled per wheel rev
     * 
     **/
    m_posLeft = K_ft_per_rev * WheelWearLeft * Kleft * kGR * leftEncoder.getPosition();
    m_posRight = K_ft_per_rev * WheelWearRight * Kright * kGR * rightEncoder.getPosition();

    /**
     * encoder.getVelocity() - motor turn in RPM ; rps = rpm/60
     *
     **/
    m_velLeft = (K_ft_per_rev * WheelWearLeft * kGR / 60.0) * leftEncoder.getVelocity();
    m_velRight = (K_ft_per_rev * WheelWearRight * kGR / 60.0) * rightEncoder.getVelocity();

    // Update the odometry in the periodic block, physical units
    m_odometry.update(readGyro(), m_posLeft, m_posRight);
  }

  /**
   * adjust the arbitrary feedforward voltage sent to the motors. This should be
   * positive, but sign is flipped when going backward.
   * 
   * Purpose is to tune to the value the motor/robot just starts to move.
   * 
   * @param deltaFF voltage to go up or down from current value
   * @return
   */
  public double adjustFeedForward(final double deltaFF) {
    arbFeedFwd = MathUtil.limit((arbFeedFwd + deltaFF), 0.0, ARBIT_FEEDFWD_MAX_VOLT);

    SmartDashboard.putNumber("/DT/limits/arbFF", arbFeedFwd);
    return arbFeedFwd;
  }

  /**
   * adjust the acceleration time - limit on how fast the output gets to max spped
   * Should be kRampRate as shown in SparkMax Client.
   * 
   * @param deltaRate (seconds) amount to add to current rate
   * @return
   */
  public double adjustAccelerationLimit(final double deltaRate) {
    slewRateLimit = MathUtil.limit((slewRateLimit + deltaRate), 0.0, DriveTrain.slewRateMax);
    // Just set the ramp limit on the masters
    leftController.setOpenLoopRampRate(slewRateLimit);
    rightController.setOpenLoopRampRate(slewRateLimit);

    // Use same rate limit on closed loop too
    leftController.setClosedLoopRampRate(slewRateLimit);
    rightController.setClosedLoopRampRate(slewRateLimit);

    SmartDashboard.putNumber("/DT/limits/slewRate", slewRateLimit);
    return slewRateLimit;
  }

  /**
   * Change the default smart current limits for drive motors. Current limits
   * pushed to all controllers.
   * 
   * @param deltaCurrent (amps) 0 - 80 amps for max power
   * @return
   */
  public int adjustCurrentLimit(final int deltaCurrent) {
    smartCurrentLimit = MathUtil.limit(smartCurrentLimit + deltaCurrent, 0, DriveTrain.smartCurrentMax);
    // final double secondaryCurrent = smartCurrentLimit * KSecondaryCurrent;

    for (final CANSparkMax c : controllers) {
      // smart current limit
      c.setSmartCurrentLimit(smartCurrentLimit);
    }
    SmartDashboard.putNumber("DT/limits/smart_I", smartCurrentLimit);
    return smartCurrentLimit;
  }

  private void resetControllers() {
    for (final CANSparkMax c : controllers) {
      c.restoreFactoryDefaults(false);
      // apply any of our controller requirements
      c.setInverted(KInvertMotor);
      c.setIdleMode(KIdleMode);
      c.clearFaults();
    }
  }

  private void saveControllers() {
    for (final CANSparkMax c : controllers) {
      c.burnFlash();
    }
  }

  // vel is ft/s positive forward
  // rotationRate deg/sec
  public void velocityArcadeDrive(final double velFps, final double rotDps) {
    // used to handle shifting request
    boolean shifting = (m_currentGear != requestedGear);
    // shifting
    if (shifting) {
      // do math on new gear to match RPM of new Vel Cmd
      m_currentGear = requestedGear;
    }

    // Spark Max uses RPM for velocity closed loop mode
    // so we need to convert ft/s to RPM command which is dependent
    // on the gear ratio.
    
    // [s/min] / [ft /wheel-rev] / [motor-rev / wheel-rev] = [motor-rpm / ft/s]
    double k = (60.0 / K_ft_per_rev) /gearbox.getGearRatio();  
    double maxSpeed = getMaxSpeed(m_currentGear);

    // limit vel to max for the gear ratio
    double vcmd = Math.copySign(MathUtil.limit(Math.abs(velFps), 0.0, maxSpeed), velFps);
    double rpm = k * vcmd;  // [rpm-mo / rpm-wheel] [rpm/rps] [ft/s] / [ft/rev]

    /**
     * Rotation controls
     */
    // Convert to rad/s split between each wheel
    double rps = Math.copySign(MathUtil.limit(Math.abs(rotDps), 0.0, maxDPS), rotDps);
    //     [mo-rpm/ ft/s]  [rad/deg] [ft] [deg/s] =  [mo-rpm/ ft/s] * [ft/s] = mo-rpm
    double vturn_rpm = k * (Math.PI / 180.0)*(0.5*WheelAxleDistance) * rps;

    // compute each wheel, pos rpm moves forward, pos turn is CCW
    double vl_rpm = applyDeadZone(rpm - vturn_rpm, RPM_DZ); // turn left, +CCW, slows left wheel
    double vr_rpm = applyDeadZone(rpm + vturn_rpm, RPM_DZ); // turn left, +CCW, speeds right wheel

    // issue all commands to the hardware
    output(vl_rpm, vr_rpm, coastMode);
  }

  /**
   * velocityTankDrive - takes wheel speeds in fps and scales to RPM and ouputs
   * those speeds.
   * 
   * Uses current gearing, but does not attempt to shift as this is most likely
   * used by trajectory control.
   * 
   * @param velLeft  [length/s] positive moves forward
   * @param velRight [length/s] positive movee forward
   */
  public void velocityTankDrive(double velLeft, double velRight) {
    // Spark Max uses RPM for velocity closed loop mode
    // so we need to convert ft/s to RPM command which is dependent
    // on the gear ratio.
    // [s/min] / [ft/wheel-rev] / [wheel-rev / motor-rev] = [mo-rev / min] / [ft/s] = [mo-rpm / ft/s]
    double kGR = (60.0 / K_ft_per_rev) / gearbox.getGearRatio();   
    
    // [mo-rpm / ft/s] * [ft/s]  = [rpm-mo]
    double rpm_l = kGR * velLeft;  
    double rpm_r = kGR * velRight; 
    // scale to rpm and ouput to contollers, no coast mode
    output(rpm_l, rpm_r, false);
  }

  /**
   * Issues all the output changes to the motor controller and gear shifter when
   * in a velocity command mode.
   * 
   * @param l_rpm     positive left side moves forward
   * @param r_rpm     positive right side moves forward
   * @param coastMode zero output, no breaks, let it roll
   */
  private void output(double l_rpm, double r_rpm, boolean coastMode) {
    if (coastMode) {
      // command doesn't need power, ok to coast, use PWM to zero power
      // with the controller setup to coast mode by default.
      leftController.set(0.0);
      rightController.set(0.0);
    } else {
      // command the velocity to the wheels, correct for wheelwear
      // also correct for any left/right motor conventions
      setReference(l_rpm * (Kleft / WheelWearLeft), leftPID);
      setReference(r_rpm * (Kright / WheelWearRight), rightPID);
    }

    shiftGears();
    dDrive.feed();
  }

  public void arcadeDrive(final double xSpeed, final double zRotation) {
    shiftGears();
    dDrive.arcadeDrive(xSpeed, zRotation, false);
  }

  public void tankDrive(final double leftSpeed, final double rightSpeed) {
    shiftGears();
    // dDrive has invert built into it, don't change with this wrapper
    dDrive.tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * getMaxSpeed based on gear and RPM limits set on motors.
   * 
   * @return max speed in ft/s
   */
  public double getMaxSpeed(Gear gear) {
    return (gear == Gear.HIGH) ? maxFPS_High : maxFPS_Low;
  }

  public double getLeftPos() {
    return m_posLeft;
  }

  public double getRightPos() {
    return m_posRight;
  }

  public double getLeftVel(final boolean normalized) {
    return (normalized) ? (m_velLeft/ maxFPS_High) : m_velLeft;
  }

  public double getRightVel(final boolean normalized) {
    return (normalized) ? (m_velRight / maxFPS_High) : m_velRight;
  }

  public double getAvgVelocity(final boolean normalized) {
    double vel = 0.5*(m_velLeft + m_velRight);
    return (normalized) ? (vel/maxFPS_High) : vel;
  }

  public void resetPosition() {
    rightController.getEncoder().setPosition(0);
    leftController.getEncoder().setPosition(0);
  }

  /**
   * shiftGears() called in coordination of velocity match
   */
  void shiftGears() {
    // do nothing if gears match
    if (requestedGear == gearbox.getCurrentGear())
      return;

    // shift based on requested
    if (requestedGear == Gear.LOW) {
      gearbox.shiftDown();
    } else {
      gearbox.shiftUp();
    }
  }

  /**
   * setCoastMode()
   */
  public void setCoastMode(boolean coast) {
    coastMode = coast;
  }

  /** Shifter is implemented by this class */
  public Shifter getShifter() {
    return this.gearbox;
  }

  /**
   * Shifter controls encapsulation
   */
  public void shiftUp() {
    requestedGear = Gear.HIGH;
  }

  public void shiftDown() {
    requestedGear = Gear.LOW;
  }

  public Gear getCurrentGear() {
    return gearbox.getCurrentGear();
  }

  public void log() {
    /**
     * SmartDashboard.putNumber("Right Velocity", getRightVel(false));
     * SmartDashboard.putNumber("Left Velocity", getLeftVel(false));
     * SmartDashboard.putNumber("Right Position", getRightPos());
     * SmartDashboard.putNumber("Left Position", getLeftPos());
     * SmartDashboard.putString("DT Command", getDefaultCommand().toString());
     * SmartDashboard.putString("Current Gear",
     * gearbox.getCurrentGear().toString());
     */
    nt_velLeft.setDouble(m_velLeft);
    nt_velRight.setDouble(m_velRight);
    nt_posLeft.setDouble(m_posLeft);
    nt_posRight.setDouble(m_posRight);
    nt_theta.setDouble(m_theta);
    nt_accelX.setDouble(m_gyro.getRawAccelX());
    nt_accelY.setDouble(m_gyro.getRawAccelY());
    nt_accelZ.setDouble(m_gyro.getRawAccelZ());
  }

  /**
   * setReference - uses physical units (rpm) for speed - volts for arbitrary feed
   * forward
   * 
   * @param rpm - revs per minute for desired
   */
  public void setReference(double rpm, CANPIDController pid) {
    // arbff compensates for min voltage needed to make robot move
    // +V moves forward, -V moves backwards
    double arbffVolts = (rpm >= 0) ? arbFeedFwd : -arbFeedFwd;

    // rpm haz a deadzone so == 0.0 is a fair test.
    if (rpm == 0.0)
      arbffVolts = 0.0;
    pid.setReference(rpm, ControlType.kVelocity, KpidSlot, arbffVolts, ArbFFUnits.kVoltage);
  }

  /**
   * Applies a symetric deadzone around zero.
   * 
   * @param value -/+ number
   * @param dz    - dead zone symetric around zero
   * @return value or zero if in deadzone
   */
  double applyDeadZone(double value, double dz) {
    double x = (Math.abs(value) < dz) ? 0.0 : value;
    return x;
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.addNumber("DT/Vel/left", () -> m_velLeft).withSize(2, 5);
    layout.addNumber("DT/Vel/right", () -> m_velRight);
    layout.addNumber("DT/pos/left", () -> m_posLeft);
    layout.addNumber("DT/pos/right", () -> m_posRight);
    layout.addNumber("DT/pos/theta", () -> m_theta);
    /*layout.addNumber("DT/HeadingDot", () -> getTurnRate());
    layout.addString("DT/gear", () -> gearbox.getCurrentGear().toString());
    layout.addNumber("DT/POSE/X", () -> getPose().getX());
    layout.addNumber("DT/POSE/Y", () -> getPose().getY());
    layout.addNumber("DT/POSE/The", () -> getPose().getRotation().getDegrees());
    layout.addNumber("DT/POSE/VoltLeft", () -> m_voltleft);
	  layout.addNumber("DT/POSE/VoltRight", () -> m_voltright);
    layout.addBoolean("DT/Motor/RightInverted?", () -> frontRight.getInverted());
    layout.addBoolean("DT/Motor/LeftInverted?", () -> frontLeft.getInverted()); */
	}

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_voltleft = Kleft * leftVolts;
    m_voltright = Kright * rightVolts;
    backLeft.setVoltage(m_voltleft);
    backRight.setVoltage(m_voltright);
    dDrive.feed();
  }

  /**
   * Returns a kinematics helper based on chassis geometry and using length units
   * robot profiled in. feet (or meters)
   * 
   * @return DifferentialDriveKinematics
   */
  @Override
  public DifferentialDriveKinematics getDriveKinematics() {
    return new DifferentialDriveKinematics(RobotPhysical.WheelAxleDistance);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_velLeft, m_velRight);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Need to use getYaw to get -180 to 180 as expected.
  Rotation2d readGyro() {
    return Rotation2d.fromDegrees(m_theta);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetPosition();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d()); // has a -1 in the interface for they gyro
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double heading = Kgyro * m_gyro.getRate();
    return heading;
  }

  void setupSimulation() {
      LinearSystem<N2, N2, N2> drivetrainPlant = 
        LinearSystemId.identifyDrivetrainSystem(
            RamseteProfile.kvVoltSecondsPerFoot,
            RamseteProfile.kaVoltSecondsSquaredPerFoot,
            1.5,  //kvVoltSecondsPerRadian,       //TODO: find real numbers
            0.3); //kaVoltSecondsSquaredPerRadian);

    // This class simulates our drivetrain's motion around the field.
    m_drivetrainSimulator = new DifferentialDrivetrainSim(
        drivetrainPlant, 
        DCMotor.getNEO(3),
        (1.0/GearShifter.K_low), 
        RobotPhysical.WheelAxleDistance, 
        (RobotPhysical.WheelDiameter/2.0)/12.0,
        VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

    // The encoder and gyro angle sims let us set simulated sensor readings
    m_leftEncoderSim = new EncoderSim2(leftController);
    m_rightEncoderSim = new EncoderSim2(rightController);
    m_gyroSim = new AHRS_GyroSim(m_gyro);

    // the Field2d class lets us visualize our robot in the simulation GUI.
    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        leftController.get() * RobotController.getBatteryVoltage(),
        -rightController.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    //we are using feet, not meeters.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  @Override
  public double getMaxVelocity() {
    return this.maxFPS_High;
  }

  @Override
  public double getMaxRotation() {
   return this.maxDPS;
  }

}
