package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.RobotPhysical.WheelAxleDistance;
import static frc.robot.Constants.RobotPhysical.WheelDiameter;
import static frc.robot.Constants.RobotPhysical.WheelWearLeft;
import static frc.robot.Constants.RobotPhysical.WheelWearRight;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.RamseteProfile;
import frc.robot.Constants.RobotPhysical;
import frc.robot.RobotContainer;
import frc.robot.sim.EncoderSim2;
import frc.robot.subsystems.ifx.DashboardUpdate;
import frc.robot.subsystems.ifx.DualDrive;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.Shifter.Gear;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.subsystems.ifx.VoltageDrive;
import frc.robot.subsystems.util.MonitoredSubsystemBase;
import frc.robot.util.misc.MathUtil;

public class VelocityDifferentialDrive_Subsystem extends MonitoredSubsystemBase
    implements Logger, DualDrive, VelocityDrive, VoltageDrive, DashboardUpdate {
  // Sign conventions, apply to encoder and command inputs
  // Brushless SparkMax cannot invert the encoders
  final double Kleft = 1.0;
  final double Kright = -1.0;
  final boolean KInvertMotor = true; // convention required in robot characterization
  final IdleMode KIdleMode = IdleMode.kBrake;
  final double Kgyro = -1.0; // ccw is positive, just like geometry class

  // Chassis Encoder
  final double kFeetPerPulse = .00057558;
  final boolean kInvertChassisLeft = true;
  final boolean kInvertChassisRight = false;

  public static class DriveSetPoints {
    public double left;
    public double right;
  }

  private NetworkTable table;
  private NetworkTableEntry nt_velLeft;
  private NetworkTableEntry nt_velRight;
  private NetworkTableEntry nt_velChassisLeft;
  private NetworkTableEntry nt_velChassisRight;
  private NetworkTableEntry nt_posLeft;
  private NetworkTableEntry nt_posRight;
  private NetworkTableEntry nt_posChassisLeft;
  private NetworkTableEntry nt_posChassisRight;
  private NetworkTableEntry nt_currentPoseX;
  private NetworkTableEntry nt_currentPoseY;
  private NetworkTableEntry nt_currentPoseYaw;
  private NetworkTableEntry nt_leftOutput;
  private NetworkTableEntry nt_rightOutput;

  // Field position
  Field2d m_field = new Field2d();

  // save a pose
  Pose2d savedPose;

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

  private final CANSparkMax[] controllers = new CANSparkMax[] { frontRight, frontLeft, backRight, backLeft, middleRight,
      middleLeft };

  // split left/right sides controller/encoder/pid
  final CANSparkMax leftController = backLeft;
  final CANSparkMax rightController = backRight;
  final CANEncoder leftEncoder = leftController.getEncoder();
  final CANEncoder rightEncoder = rightController.getEncoder();
  final CANPIDController leftPID = leftController.getPIDController();
  final CANPIDController rightPID = rightController.getPIDController();

  final Encoder leftChassisEncoder = new Encoder(DigitalIO.LEFT_CHASSIS_ENCODER_A, DigitalIO.LEFT_CHASSIS_ENCODER_B);
  final Encoder rightChassisEncoder = new Encoder(DigitalIO.RIGHT_CHASSIS_ENCODER_A, DigitalIO.RIGHT_CHASSIS_ENCODER_B);

  // controls which encoders to use for position estimates on field in getPose()
  boolean m_useChassisEncoders = false;

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
  final Shifter gearbox;
  Gear requestedGear; // where driver or external logic wants the gear
  boolean coastMode = false; // attempt to coast when Vcmd > Vrequest

  // Save our commanded wheelspeeds and current speeds
  final DifferentialDriveWheelSpeeds m_commandedWheelSpeeds = new DifferentialDriveWheelSpeeds();

  // measurements, taken in periodic(), robot coordinates
  final DifferentialDriveWheelSpeeds m_wheelSpeeds = new DifferentialDriveWheelSpeeds();
  double m_posLeft = 0.0; // feet, positive is forward distance, since encoder reset
  double m_posRight = 0.0; // feet, positive is forward distance, since encoder reset
  Gear m_currentGear; // high/low
  double m_posChassisLeft = 0.0; // feet, positive forward, since encoder reset
  double m_posChassisRight = 0.0; // feet, positive forward, since encoder reset
  double m_velChassisLeft;
  double m_velChassisRight;

  double m_voltleft; // save voltages that we send to motor
  double m_voltright;
  Supplier<Double> m_heading_compensator; // radians/s

  // setpoints for left/right wheel speeds [rpm] Sparkmax
  final DriveSetPoints sp_rpm = new DriveSetPoints();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveOdometry m_odometry_chassis;

  // nav sensors
  Sensors_Subsystem nav;

  // Simulation - these classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private EncoderSim2 m_leftEncoderSim;
  private EncoderSim2 m_rightEncoderSim;

  // list of positions for recording path
  public List<Translation2d> PositionList = new ArrayList<Translation2d>();
  private boolean recordPositionOn = false;
  public Pose2d recordingPoseStart;
  public Pose2d recordingPoseEnd;

  public VelocityDifferentialDrive_Subsystem(final Shifter gear) {
    // save scaling factors, they are required to use SparkMax in Vel mode
    gearbox = gear;
    requestedGear = gearbox.getCurrentGear();

    // get the nav sensor subsystem for odometry reporting
    nav = RobotContainer.getInstance().sensors;
    setHeadingCompensator(null);

    // direct networktables logging
    table = NetworkTableInstance.getDefault().getTable("Drivetrain");
    nt_velLeft = table.getEntry("VelLeft/value");
    nt_velRight = table.getEntry("VelRight/value");
    nt_velChassisLeft = table.getEntry("VelChassisLeft");
    nt_velChassisRight = table.getEntry("VelChassisRight");
    nt_posLeft = table.getEntry("PosLeft");
    nt_posRight = table.getEntry("PosRight");
    nt_posChassisLeft = table.getEntry("PosChassisLeft");
    nt_posChassisRight = table.getEntry("PosChassisRight");

    nt_currentPoseX = table.getEntry("CurrentX");
    nt_currentPoseY = table.getEntry("CurrentY");
    nt_currentPoseYaw = table.getEntry("CurrentYaw");
    nt_rightOutput = table.getEntry("MotorOutRight");
    nt_leftOutput = table.getEntry("MotorOutLeft");

    SmartDashboard.putData("Field", m_field);

    // setup physical units - chassis * gearbox (rev per minute)
    K_low_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.LOW) / 60; // rpm/60 rps
    K_high_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.HIGH) / 60;

    leftChassisEncoder.setDistancePerPulse(kFeetPerPulse);
    leftChassisEncoder.setReverseDirection(kInvertChassisLeft);
    rightChassisEncoder.setDistancePerPulse(kFeetPerPulse);
    rightChassisEncoder.setReverseDirection(kInvertChassisRight);

    // Speed setting may be updated via UX, but set defaults
    calcSpeedSettings();

    // setup SparkMax controllers, sets left and right masters
    configureControllers();

    // create a dDrive to support other modes besides velocity command
    dDrive = new DifferentialDrive(leftController, rightController);
    dDrive.setSafetyEnabled(DriveTrain.safetyEnabled);

    // init simulation iff needed
    if (RobotBase.isSimulation())
      setupSimulation();

    // clear our starting position.
    resetPosition();
    m_odometry = new DifferentialDriveOdometry(nav.getRotation2d());
    m_odometry_chassis = new DifferentialDriveOdometry(nav.getRotation2d());

    savedPose = m_odometry.getPoseMeters(); // set savedPose to starting position initally
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

    // master controller have faster CAN Timing
    setMasterControlerTiming(leftController);
    setMasterControlerTiming(rightController);

    // zero adjust will set the default limits for accel and currents
    adjustAccelerationLimit(0.0);
    adjustFeedForward(0.0);
    adjustCurrentLimit(0);

    // burn the default value in case of brown-out
    saveControllers();
  }

  @Override
  public void monitored_periodic() {
    m_currentGear = gearbox.getCurrentGear();
    double kGR = gearbox.getGearRatio();
    /**
     * Chassis encoders - gear and wheeldiam set in scaler for getDistance()
     */
    m_posChassisLeft = WheelWearLeft * leftChassisEncoder.getDistance();
    m_posChassisRight = WheelWearRight * rightChassisEncoder.getDistance();
    m_velChassisLeft = WheelWearLeft * leftChassisEncoder.getRate();
    m_velChassisRight = WheelWearRight * rightChassisEncoder.getRate();

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
    m_wheelSpeeds.leftMetersPerSecond = (Kleft * K_ft_per_rev * WheelWearLeft * kGR / 60.0) * leftEncoder.getVelocity();
    m_wheelSpeeds.rightMetersPerSecond = (Kright * K_ft_per_rev * WheelWearRight * kGR / 60.0)  * rightEncoder.getVelocity();

    // Update the odometry in the periodic block, physical units, update field
    m_odometry.update(nav.getRotation2d(), m_posLeft, m_posRight);
    m_odometry_chassis.update(nav.getRotation2d(), m_posChassisLeft, m_posChassisRight);
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  // Record position
  public void addPosition() {
    PositionList.add(new Translation2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY()));
  }

  public void turnPositionRecordingOn() {
    recordPositionOn = true;
    recordingPoseStart = m_odometry.getPoseMeters();
  }

  public void turnPositionRecordingOff() {
    recordPositionOn = false;
    recordingPoseEnd = m_odometry.getPoseMeters();
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
    for (final CANSparkMax c : controllers) {
      // smart current limit
      c.setSmartCurrentLimit(smartCurrentLimit);
    }
    return smartCurrentLimit;
  }

  private void resetControllers() {
    for (final CANSparkMax c : controllers) {
      c.restoreFactoryDefaults(false);
      // apply any of our controller requirements
      c.setInverted(KInvertMotor);
      c.setIdleMode(KIdleMode);
      c.clearFaults();

      // lower CAN message rates, for everyone. Masters will get set higher later
      c.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200); // applied output (norm 10ms)
      c.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500); // motor Vel,T, Volt (norm 20ms)
      c.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // motor pos (norm 20ms)
    }
  }

  /**
   * Master controllers need to report back faster or at normal rate
   * 
   * @param c SparkMax controller used as a master
   */
  private void setMasterControlerTiming(CANSparkMax c) {
    c.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // applied output (norm 10ms)
    c.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10); // motor Vel,T, Volt (norm 20ms)
    c.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // motor pos (norm 20ms)
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
    double kGR = (60.0 / K_ft_per_rev) / gearbox.getGearRatio();
    double maxSpeed = getMaxSpeed(m_currentGear);

    // limit vel to max for the gear ratio
    double vcmd = Math.copySign(MathUtil.limit(Math.abs(velFps), 0.0, maxSpeed), velFps);
    double rpm = kGR * vcmd; // [rpm-mo / rpm-wheel] [rpm/rps] [ft/s] / [ft/rev]

    /**
     * Rotation controls
     */
    // Convert to rad/s split between each wheel
    double rps = Math.copySign(MathUtil.limit(Math.abs(rotDps), 0.0, maxDPS), rotDps);
    rps = rps + m_heading_compensator.get();

    // [mo-rpm/ ft/s] [rad/deg] [ft] [deg/s] = [mo-rpm/ ft/s] * [ft/s] = mo-rpm
    double vturn_rpm = kGR * (Math.PI / 180.0) * (0.5 * WheelAxleDistance) * rps;

    // compute each wheel, pos rpm moves forward, pos turn is CCW
    double vl_rpm = applyDeadZone(rpm - vturn_rpm, RPM_DZ); // turn left, +CCW, slows left wheel
    double vr_rpm = applyDeadZone(rpm + vturn_rpm, RPM_DZ); // turn left, +CCW, speeds right wheel

    // issue all commands to the hardware
    output(vl_rpm, vr_rpm, coastMode, kGR);
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
    // [s/min] / [ft/wheel-rev] / [wheel-rev / motor-rev] = [mo-rev / min] / [ft/s]
    // = [mo-rpm / ft/s]
    double kGR = (60.0 / K_ft_per_rev) / gearbox.getGearRatio();
    double rps = m_heading_compensator.get();
    double vturn_rpm = (Math.PI / 180.0) * (0.5 * WheelAxleDistance) * rps;

    // [mo-rpm / ft/s] * [ft/s] = [rpm-mo]
    double rpm_l = kGR * (velLeft - vturn_rpm);
    double rpm_r = kGR * (velRight + vturn_rpm);
    // scale to rpm and ouput to contollers, no coast mode
    output(rpm_l, rpm_r, false, kGR);
  }

  /**
   * 
   * sets supplier for heading compensation in degs/sec
   * 
   * @param headingDotComp degs/sec to turn based on external system like
   *                       limelight or gyro
   */
  public void setHeadingCompensator(Supplier<Double> headingDotComp) {
    if (null == headingDotComp) {
      m_heading_compensator = this::defaultHeadingCompensator;
    }
  }

  /**
   * No heading compensation - use by default
   * 
   * @return 0.0
   */
  double defaultHeadingCompensator() {
    return 0.0;
  }

  /**
   * Issues all the output changes to the motor controller and gear shifter when
   * in a velocity command mode.
   * 
   * @param l_rpm     positive left side moves forward
   * @param r_rpm     positive right side moves forward
   * @param coastMode zero output, no breaks, let it roll
   * @param kGR       gearRatio [mo-rpm / ft/s]
   */
  private void output(double l_rpm, double r_rpm, boolean coastMode, double kGR) {
    if (coastMode) {
      // command doesn't need power, ok to coast, use PWM to zero power
      // with the controller setup to coast mode by default.
      leftController.set(0.0);
      rightController.set(0.0);
    } else {
      // command the velocity to the wheels, correct for wheelwear
      // also correct for any left/right motor conventions
      sp_rpm.left = l_rpm * (Kleft / WheelWearLeft);
      sp_rpm.right = r_rpm * (Kright / WheelWearRight);

      // save commanded wheel speeds - [ft/s]
      m_commandedWheelSpeeds.leftMetersPerSecond = sp_rpm.left / kGR; // not meters, we use ft/s
      m_commandedWheelSpeeds.rightMetersPerSecond = sp_rpm.right / kGR;

      // arbff compensates for min voltage needed to make robot move
      // +V moves forward, -V moves backwards
      double arbffVoltsLeft = (sp_rpm.left > 0) ? arbFeedFwd : -arbFeedFwd;
      double arbffVoltsRight = (sp_rpm.right > 0) ? arbFeedFwd : -arbFeedFwd;

      // rpm haz a deadzone so rpm == 0.0 is a fair test.
      if (sp_rpm.left == 0.0)   arbffVoltsLeft = 0.0;
      if (sp_rpm.right == 0.0)  arbffVoltsRight = 0.0;

      leftPID.setReference(sp_rpm.left, ControlType.kVelocity, KpidSlot, arbffVoltsLeft, ArbFFUnits.kVoltage);
      rightPID.setReference(sp_rpm.right, ControlType.kVelocity, KpidSlot, arbffVoltsRight, ArbFFUnits.kVoltage);
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
    return (normalized) ? (m_wheelSpeeds.leftMetersPerSecond / maxFPS_High) : m_wheelSpeeds.leftMetersPerSecond;
  }

  public double getRightVel(final boolean normalized) {
    return (normalized) ? (m_wheelSpeeds.rightMetersPerSecond / maxFPS_High) : m_wheelSpeeds.rightMetersPerSecond;
  }

  public double getAvgVelocity(final boolean normalized) {
    double vel = 0.5 * (m_wheelSpeeds.leftMetersPerSecond + m_wheelSpeeds.rightMetersPerSecond);
    return (normalized) ? (vel / maxFPS_High) : vel;
  }

  public DriveSetPoints getVelocitySetpoints() {
    return sp_rpm;
  }

  public void resetPosition() {
    // SparkMax encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // Chassis encoders
    leftChassisEncoder.reset();
    rightChassisEncoder.reset();
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

    // encoder vel and positions
    nt_velLeft.setDouble(m_wheelSpeeds.leftMetersPerSecond); // ft/s
    nt_velRight.setDouble(m_wheelSpeeds.rightMetersPerSecond); // ft/s
    nt_posLeft.setDouble(m_posLeft);
    nt_posRight.setDouble(m_posRight);

    nt_posChassisLeft.setDouble(m_posChassisLeft);
    nt_posChassisRight.setDouble(m_posChassisRight);

    nt_velChassisLeft.setDouble(m_velChassisLeft);
    nt_velChassisRight.setDouble(m_velChassisRight);
    // pose
    nt_currentPoseX.setDouble(m_odometry.getPoseMeters().getX());
    nt_currentPoseY.setDouble(m_odometry.getPoseMeters().getY());
    nt_currentPoseYaw.setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());

    // motor outputs
    nt_leftOutput.setDouble(leftController.getAppliedOutput());
    nt_rightOutput.setDouble(rightController.getAppliedOutput());

    if (recordPositionOn)
      addPosition();
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
    /*
     * layout.addNumber("DT/Vel/left", () -> m_velLeft).withSize(2, 5);
     * layout.addNumber("DT/Vel/right", () -> m_velRight);
     * layout.addNumber("DT/pos/left", () -> m_posLeft);
     * layout.addNumber("DT/pos/right", () -> m_posRight);
     * 
     * layout.addString("DT/gear", () -> gearbox.getCurrentGear().toString());
     * layout.addNumber("DT/POSE/X", () -> getPose().getX());
     * layout.addNumber("DT/POSE/Y", () -> getPose().getY());
     * layout.addNumber("DT/POSE/The", () -> getPose().getRotation().getDegrees());
     * layout.addNumber("DT/POSE/VoltLeft", () -> m_voltleft);
     * layout.addNumber("DT/POSE/VoltRight", () -> m_voltright);
     * layout.addBoolean("DT/Motor/RightInverted?", () -> frontRight.getInverted());
     * layout.addBoolean("DT/Motor/LeftInverted?", () -> frontLeft.getInverted());
     */
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
    return m_wheelSpeeds;
  }

  public DifferentialDriveWheelSpeeds getCommandedWheelSpeeds() {
    return m_commandedWheelSpeeds;
  }

  /**
   * 
   * @param chassis_enc  - true use encoders on chassis
   *                     - false use the SparkMax off the CAN bus
   */
  public void useChassisEncoders(Boolean chassis_enc) {
    m_useChassisEncoders = chassis_enc;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return (m_useChassisEncoders) ? m_odometry_chassis.getPoseMeters() : m_odometry.getPoseMeters();
  }

  // update the savedPose on demand
  public void savePose() {
    savedPose = m_odometry.getPoseMeters();
    return;
  }

  public Pose2d getSavedPose() {
    // System.out.println("Saved X:" + savedPose.getX());
    // System.out.println("Saved Y:" + savedPose.getY());
    return savedPose;
  }

  /**
   * Resets the odometry to the specified pose. X, Y are set as is the expected
   * rotation. The gyro is sampled and used as an offset internally.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetPosition();
    // dpl - 3/23/21 not sure if this should be here - nav.reset();
    m_odometry.resetPosition(pose, nav.getRotation2d());
    m_odometry_chassis.resetPosition(pose, nav.getRotation2d());
  }

  void setupSimulation() {
    LinearSystem<N2, N2, N2> drivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
        RamseteProfile.kvVoltSecondsPerFoot, RamseteProfile.kaVoltSecondsSquaredPerFoot, 1.5, // kvVoltSecondsPerRadian,
                                                                                              // //TODO: find real
                                                                                              // numbers
        0.3); // kaVoltSecondsSquaredPerRadian);

    // This class simulates our drivetrain's motion around the field.
    m_drivetrainSimulator = new DifferentialDrivetrainSim(drivetrainPlant, DCMotor.getNEO(3), (1.0 / GearShifter.K_low),
        RobotPhysical.WheelAxleDistance, (RobotPhysical.WheelDiameter / 2.0) / 12.0,
        VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

    // The encoder and gyro angle sims let us set simulated sensor readings
    m_leftEncoderSim = new EncoderSim2(leftController);
    m_rightEncoderSim = new EncoderSim2(rightController);
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(leftController.get() * RobotController.getBatteryVoltage(),
        -rightController.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    // we are using feet, not meeters.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION
   * ONLY! If you want it to work elsewhere, use the code in
   * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
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
