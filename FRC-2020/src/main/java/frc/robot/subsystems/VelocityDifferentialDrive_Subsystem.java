package frc.robot.subsystems;

import static frc.robot.Constants.RobotPhysical.WheelAxelDistance;
import static frc.robot.Constants.RobotPhysical.WheelDiameter;
import static frc.robot.Constants.RobotPhysical.WheelWearLeft;
import static frc.robot.Constants.RobotPhysical.WheelWearRight;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.RamseteProfile;
import frc.robot.subsystems.ifx.DualDrive;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.subsystems.ifx.VoltageDrive;
import frc.robot.util.misc.MathUtil;
import frc.robot.util.misc.PIDFController;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements Logger, DualDrive, VelocityDrive, 
                VoltageDrive, Shifter {
  // Sign conventions, apply to encoder and command inputs
  // Brushless SparkMax cannot invert the encoders
  final double Kleft = 1.0;
  final double Kright = -1.0;
  final boolean KInvertMotor = true; // convention required in robot characterization 
  final IdleMode KIdleMode = IdleMode.kCoast;
  final double Kgyro = -1.0;         // ccw is positive, just like geometry class
  
  // we only use this one PID slot for the drive lead controllers
  final int KpidSlot = 0;

	// Current Limits
	private int smartCurrentLimit = DriveTrain.smartCurrentLimit; // amps
	// private final double KSecondaryCurrent = 1.40; // set secondary current based on smart current

	// Phyical units deadzone
	private final double RPM_DZ = 2.0;

	// Acceleration limits
	private double slewRateLimit = DriveTrain.slewRateLimit;   // seconds to max speed/power

	// Chasis details
	private final double K_ft_per_rev = Math.PI * WheelDiameter / 12.0; // feet/rev
	private final double K_low_fps_rpm; // Low gear ft/s / rpm of motor shaft
	private final double K_high_fps_rpm; // High gear ft/s /rpm of motor shaft

	// CANSpark Max will be used 3 per side, 2 folowing the leader
	private final CANSparkMaxLowLevel.MotorType MT = CANSparkMaxLowLevel.MotorType.kBrushless;
	private final CANSparkMax frontRight = new CANSparkMax(CAN.FR_SMAX, MT);
	private final CANSparkMax frontLeft = new CANSparkMax(CAN.FL_SMAX, MT);
	private final CANSparkMax backRight = new CANSparkMax(CAN.BR_SMAX, MT);
	private final CANSparkMax backLeft = new CANSparkMax(CAN.BL_SMAX, MT);
	private final CANSparkMax middleRight = new CANSparkMax(CAN.MR_SMAX, MT);
	private final CANSparkMax middleLeft = new CANSparkMax(CAN.ML_SMAX, MT);

	private final CANSparkMax[] controllers = new CANSparkMax[] { frontRight, frontLeft, backRight, backLeft,
			middleRight, middleLeft };

	// VelController can use either Velocity mode or dutycycle modes and is wrapper
	// around CANSparkMax. These get setup after factory resets.
	final CANSparkMax leftController = backLeft; 
  final CANSparkMax rightController = backRight;
  final CANPIDController leftPID = leftController.getPIDController();
	final CANPIDController rightPID = rightController.getPIDController();
	
  //  Voltage to get robot to move.
  // kS - taken from the Drive Characterization
  //    - for reference, 0.15 was enough to move 2019 arm-bot chassis
  private final double ARBIT_FEEDFWD_MAX_VOLT = 1.5;     //volts max allowed
	private double arbFeedFwd = RamseteProfile.ksVolts;    //current value

	// Calculated based on desired low-gear max ft/s
	private final double maxFPS_High;  // <input>
	private final double maxFPS_Low;   // using HIGH gear max RPM
	private final double maxDPS;       // max rotation in deg/sec around Z axis
	//private final double maxRPM_Low;   // max motor RPM low & high
	private final double maxRPM_High;  // max motor RPM low & high

  // drivetrain & gear objects 
	final DifferentialDrive dDrive;
	Shifter gearbox;
	Gear requestedGear;                // where driver or external logic wants the gear
	boolean coastMode = false;         // attempt to coast when Vcmd > Vrequest
	
	//measurements, taken in periodic(), robot coordinates
	double m_velLeft  = 0.0;  //fps, positive moves robot forward
  double m_velRight = 0.0;  //fps, positive moves robot forward
  double m_posLeft  = 0.0;  //feet, positive is forward distance, since encoder reset
  double m_posRight = 0.0;  //feet, positive is forward distance, since encoder reset
	Gear m_currentGear;       //high/low
	double m_theta;           //heading   

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	public VelocityDifferentialDrive_Subsystem(final Shifter gear) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		gearbox = gear;
		maxDPS = DriveTrain.maxRotDPS;
		requestedGear = gearbox.getCurrentGear();
		maxFPS_High = DriveTrain.maxFPS;

		// setup physical units - chassis * gearbox  (rev per minute)
		K_low_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.LOW) / 60;    //rpm/60 rps
		K_high_fps_rpm = K_ft_per_rev * gearbox.getGearRatio(Gear.HIGH) / 60;
    
    // compute max RPM for motors for high and low gears
		maxRPM_High = (maxFPS_High / K_high_fps_rpm); 
		maxFPS_Low = (DriveTrain.motorMaxRPM * K_low_fps_rpm);

		// check we can hit max requested speed in high gear
		if (maxRPM_High > DriveTrain.motorMaxRPM) {
			System.out.println("Warning: targeted maxFPS not reachable. maxFPS= " + 
				(DriveTrain.motorMaxRPM * K_high_fps_rpm));
		}

		// setup SparkMax controllers, sets left and right masters
		configureControllers();

    // create a dDrive to support other modes besides velocity command
		dDrive = new DifferentialDrive(leftController, rightController);
		dDrive.setSafetyEnabled(DriveTrain.safetyEnabled);

		// this may cause a 1-2 second delay - robot can't move during period
    m_gyro.calibrate();
    while (m_gyro.isCalibrating()) { // wait to zero yaw if calibration is still running
      try {
        Thread.sleep(250);
        System.out.println("calibrating gyro");
      } catch (InterruptedException e) {  /*do nothing */  }
    }
    System.out.println("Pre-Zero yaw:" + m_gyro.getYaw());
		m_gyro.reset(); //should zero yaw but not working.
		System.out.println("Post-Zero yaw:" + m_gyro.getYaw());

    // clear our starting position.
    resetPosition();
		m_odometry = new DifferentialDriveOdometry(readGyro());
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
    configurePID(leftPID, DriveTrain.pidValues);
    configurePID(rightPID, DriveTrain.pidValues);

		// zero adjust will set the default limits for accel and currents
    adjustAccelerationLimit(0.0);
    adjustFeedForward(0.0);
		adjustCurrentLimit(0);
    
		// burn the default value in case of brown-out
		saveControllers();
	}


	@Override
	public void periodic() {
		// read required sensor and system values for the frame
		m_velLeft = getLeftVel(false);
		m_velRight = getRightVel(false);
		m_posLeft = getLeftPos();
		m_posRight = getRightPos();
		m_currentGear = gearbox.getCurrentGear(); 
		m_theta = Kgyro*m_gyro.getYaw();

		// Update the odometry in the periodic block, physical units
		m_odometry.update(readGyro(), m_posLeft, m_posRight);
	}


	/**
	 * adjust the arbitrary feedforward voltage sent to the motors.  This should
	 * be positive, but sign is flipped when going backward.
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
	 * Change the default smart current limits for drive motors.
	 * Current limits pushed to all controllers.
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
      //apply any of our controller requirements
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
		//shifting 
		if (shifting) {
			// do math on new gear to match RPM of new Vel Cmd
			m_currentGear = requestedGear;   
		}

		// Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
		double k = 1.0 / getFPSperRPM(m_currentGear);   //k [rpm/fps]
		double maxSpeed = maxFPS_High;   //getMaxSpeed(currentGear);
		
		// limit vel to max for the gear ratio
		double vcmd = Math.copySign(MathUtil.limit(Math.abs(velFps), 0.0, maxSpeed), velFps);
		double rpm = vcmd * k;

		/**
		 * Rotation controls
		 */
		// Convert to rad/s split between each wheel
		double rps = Math.copySign(MathUtil.limit(Math.abs(rotDps), 0.0, maxDPS), rotDps);
		rps *= (Math.PI / 180.0);
		double vturn_rpm = (rps * WheelAxelDistance) * k;

		// compute each wheel, pos rpm moves forward, pos turn is CCW
		double vl_rpm = applyDeadZone(rpm - vturn_rpm, RPM_DZ);  //turn left, +CCW, slows left wheel
    double vr_rpm = applyDeadZone(rpm + vturn_rpm, RPM_DZ);  //turn left, +CCW, speeds right wheel
    
    // issue all commands to the hardware
    output( vl_rpm, vr_rpm, coastMode);
	}

  /**
   *  velocityTankDrive - takes wheel speeds in fps and scales
   *    to RPM and ouputs those speeds. 
   * 
   *  Uses current gearing, but does not attempt to shift as this is most likely
   *  used by trajectory control.
   * 
   * @param velLeft      [length/s]    positive moves forward
   * @param velRight     [length/s]    positive movee forward
   */
  public void velocityTankDrive(double velLeft, double velRight) {
    // Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
    double k = 1.0 / getFPSperRPM(m_currentGear);   // k = [RPM/fps]
    
    //scale to rpm and ouput to contollers, no coast mode
    output(k*velLeft, k*velRight, false);
  }

  /**
   * Issues all the output changes to the motor controller and gear shifter
   * when in a velocity command mode.
   * 
   * @param l_rpm   positive left side moves forward
   * @param r_rpm   positive right side moves forward
   * @param coastMode  zero output, no breaks, let it roll
   */
  private void output(double l_rpm, double r_rpm, boolean coastMode) {
    if (coastMode) {
			// command doesn't need power, ok to coast, should use PWM and zero power
			// with the controller setup to coast mode by default.
			leftController.set(0.0);
			rightController.set(0.0);
    }
    else {
      // command the velocity to the wheels, correct for wheelwear
      // also correct for any left/right motor conventions
			setReference(l_rpm/WheelWearLeft*Kleft, leftPID);
			setReference(r_rpm/WheelWearRight*Kright, rightPID);
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
	public double getMaxSpeed( Gear gear) {
		return (gear == Gear.HIGH) ? maxFPS_High : maxFPS_Low;
	}

	private double getFPSperRPM(Gear gear) {
		return (gear == Gear.HIGH) ?  K_high_fps_rpm : K_low_fps_rpm;
	}


	public double getLeftPos() {
		// postion is in units of revs
		double kft_rev = Kleft*getFPSperRPM(getCurrentGear())*60.0;
		double ft =  kft_rev * leftController.getEncoder().getPosition();  
		return ft*WheelWearLeft;  // account for calibration, we didn't go as far as we think
	}

	public double getRightPos() {
		// postion is in units of revs
		double kft_rev = Kright*getFPSperRPM(getCurrentGear())*60.0;
		double ft = kft_rev * rightController.getEncoder().getPosition();
		return ft*WheelWearRight;
	}

	public double getLeftVel(final boolean normalized) {
		double vel =  Kleft * leftController.get();           
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel*WheelWearLeft;
	}

	public double getRightVel(final boolean normalized) {
		double vel = Kright * rightController.get();
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel*WheelWearRight;
	}

	public double getAvgVelocity(final boolean normalized) {
    double vel = 0.5*(Kright * rightController.get() * WheelWearRight 
                     + Kleft * leftController.get() * WheelWearLeft);
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel;
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
		if (requestedGear == gearbox.getCurrentGear()) return;
		
		// shift based on requested
		if (requestedGear == Gear.LOW) {
			gearbox.shiftDown();
		} 
		else {
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
		return this;
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
    }

    void configurePID(CANPIDController hwpid, PIDFController pidf) {
      // set PID coefficients
      hwpid.setP(pidf.getP(), KpidSlot);
      hwpid.setI(pidf.getI(), KpidSlot);
      hwpid.setD(pidf.getD(), KpidSlot);
      hwpid.setIZone(0.0, KpidSlot);
      hwpid.setFF(pidf.getF(), KpidSlot);
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
	 *  Implement Shifter interface by deferring to the gearbox being used.
	 */
	@Override
	public boolean isAutoShiftEnabled() {
		return gearbox.isAutoShiftEnabled();
	}

	@Override
	public boolean enableAutoShift() {
		return gearbox.enableAutoShift();
	}

	@Override
	public boolean disableAutoShift() {
		return gearbox.disableAutoShift();
	}


	/**
	 * Applies a symetric deadzone around zero.
	 * 
	 * @param value  -/+ number 
	 * @param dz     - dead zone symetric around zero
	 * @return value or zero if in deadzone
	 */
	double applyDeadZone(double value, double dz) {
		double x = (Math.abs(value) < dz) ? 0.0 : value;
		return x;
	}

	public void addDashboardWidgets(ShuffleboardLayout layout) {
		layout.addNumber("/DT/Vel/left", () -> m_velLeft).withSize(2, 2);
    layout.addNumber("/DT/Vel/right", () -> m_velRight);
    layout.addNumber("/DT/pos/left", () -> m_posLeft);
    layout.addNumber("/DT/pos/right", () -> m_posRight);
    layout.addNumber("/DT/pos/theta", () -> m_theta);
		layout.addString("/DT/gear", () -> gearbox.getCurrentGear().toString());
		
	}

	  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    backLeft.setVoltage(Kleft* leftVolts);
    backRight.setVoltage(Kright * rightVolts);
    dDrive.feed();
  }

  /**
   * Returns a kinematics helper based on chassis geometry
   *  and using length units robot profiled in.  feet (or meters)
   * 
   * @return DifferentialDriveKinematics  
   */
  @Override
  public DifferentialDriveKinematics getDriveKinematics() {
    return new DifferentialDriveKinematics(WheelAxelDistance);
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

  //Need to use getYaw to get -180 to 180 as expected.
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
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());  // has a -1 in the interface for they gyro
  }

  @Override
  public double getGearRatio(Gear g) {
    return gearbox.getGearRatio(g);
  }

  @Override
  public double getGearRatio() {
    return gearbox.getGearRatio();
  }

}
