package frc.robot.subsystems;

import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.*;
import frc.robot.util.misc.MathUtil;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.SpeedController;

import static frc.robot.Constants.*;

import java.util.concurrent.TimeUnit;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements Logger, DualDrive, VelocityDrive, Shifter {
	// Current Limits
	private final int SMARTCURRENT_MAX = 60;
	private int smartCurrentLimit = 35; // amps
	// private final double KSecondaryCurrent = 1.40; // set secondary current based
	// on smart current

	// Phyical units deadzone
	private final double RPM_DZ = 2.0;

	// Acceleration limits
	private final double RATE_MAX_SECONDS = 2;
	private double rateLimit = 0.9; // seconds to max speed/power

	// Chasis details
	public final double WHEEL_RADIUS = 7.5/2.0; // inches
	private final double WHEEL_AXLE_DIST = 25.5 / 12.0; // feet
	private final double K_ft_per_rev = (2.0 * Math.PI * WHEEL_RADIUS) / 12.0; // feet/rev
	private final double K_low_fps_rpm; // Low gear ft/s / rpm of motor shaft
	private final double K_high_fps_rpm; // High gear ft/s /rpm of motor shaft

	// CANSpark Max will be used 3 per side, 2 folowing the leader
	private final CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID,
			CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);

	private final CANSparkMax[] controllers = new CANSparkMax[] { frontRight, frontLeft, backRight, backLeft,
			middleRight, middleLeft };

	private final AHRS navX = new AHRS();

	// VelController can use either Velocity mode or dutycycle modes and is wrapper
	// around CANSparkMax. These get setup after factory resets.
	private VelController leftController; 
	private VelController rightController;

	// PID coefficients
	private final double kP = 0.00005;
	private final double kI = 0.000006;
	private final double kD = 0;
	private final double kIz = 0.01;
	private final double kFF = 0.00025;
	private final double kMaxOutput = 1;
	private final double kMinOutput = -1;
	private final int pidSlot = 0;
	
	// likely not needed because it is so low to get robot to move.
	private final double FEEDFWD_MAX_VOLT = 0.0;   //0.15 was enough to move Alfred.
	private double feedFwd = 0.0;

	// Calculated based on desired low-gear max ft/s
	private final double maxFPS_High;  // Assigned
	private final double maxFPS_Low;   // using HIGH gear RPM
	private final double maxDPS;       // max rotation in deg/sec around Z axis
	//private final double maxRPM_Low;   // max motor RPM low & high
	private final double maxRPM_High;  // max motor RPM low & high

	private final DifferentialDrive dDrive;
	private final DifferentialDriveOdometry odometry;
	public static final double trackWidthMeters = .61;
	public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(trackWidthMeters);
	public static final TrajectoryConfig TRAJ_CONFIG = new TrajectoryConfig(3, 3).setKinematics(DRIVE_KINEMATICS);

	private GearShifter gearbox;
	private Gear requestedGear; 
	private boolean coastMode = false;
	

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear, final double maxFPS, final double maxDPS) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		this.gearbox = gear;
		this.maxDPS = maxDPS;
		this.requestedGear = gearbox.getCurrentGear();
		this.maxFPS_High = maxFPS;

		// setup physical units - chassis * gearbox
		K_low_fps_rpm = K_ft_per_rev * gearbox.K_low / 60;
		K_high_fps_rpm = K_ft_per_rev * gearbox.K_high / 60;
		// compute max RPM for motors
		maxRPM_High = (maxFPS / K_high_fps_rpm); 
		maxFPS_Low = (maxRPM_High * K_low_fps_rpm);

		// setup SparkMax controllers, sets left and right masters
		configureControllers();

		// zero adjust will set the default limits
		adjustAccelerationLimit(0.0);
		adjustCurrentLimit(0);

		dDrive = new DifferentialDrive(leftController, rightController);
		odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
		dDrive.setSafetyEnabled(false);
	}

	public void periodic() {
		//TODO: Get unit conversion
		odometry.update(Rotation2d.fromDegrees(getHeading()), leftController.getPosition(), rightController.getPosition());
	}

	/**
	 * hides some of the ugly setups like delays between programming.
	 */
	void configureControllers() {
		// factory reset
		resetControllers();

		// velocity setup - using RPM speed controller, sets pid
		this.leftController = new VelController(backLeft);
		this.rightController = new VelController(backRight);

		// Have motors follow to use Differential Drive
		middleRight.follow(backRight);
		frontRight.follow(backRight);
		// Left side
		middleLeft.follow(backLeft);
		frontLeft.follow(backLeft);

		// zero adjust will set the default limits for accel and currents
		adjustAccelerationLimit(0.0);
		adjustCurrentLimit(0);

		// burn the default value incase of brown-out
		saveControllers();
	}

	/**
	 * adjust the feedforward voltage sent to the motors.  This should
	 * be positive, but sign is flipped when going backward.
	 * 
	 * Tune to the value the motor/robot just starts to move.
	 * 
	 * @param deltaFF voltage to go up or down from current value
	 * @return
	 */
	public double adjustFeedForward(final double deltaFF) {
		feedFwd = MathUtil.limit((feedFwd + deltaFF), 0.0, FEEDFWD_MAX_VOLT);

		SmartDashboard.putNumber("motorFF", feedFwd);
		return feedFwd;
	}



	/**
	 * adjust the acceleration time - limit on how fast the output gets to max spped
	 * Should be kRampRate as shown in SparkMax Client.
	 * 
	 * @param deltaRate (seconds) amount to add to current rate
	 * @return
	 */
	public double adjustAccelerationLimit(final double deltaRate) {
		rateLimit = MathUtil.limit((rateLimit + deltaRate), 0.0, RATE_MAX_SECONDS);
		// Just set the ramp limit on the masters
		leftController.controller.setOpenLoopRampRate(rateLimit);
		rightController.controller.setOpenLoopRampRate(rateLimit);

		// Use same rate limit on closed loop too
		leftController.controller.setClosedLoopRampRate(rateLimit);
		rightController.controller.setClosedLoopRampRate(rateLimit);

		SmartDashboard.putNumber("motorRate", rateLimit);
		return rateLimit;
	}

	/**
	 * Change the default smart current limits for drive motors.
	 * Current limits pushed to all controllers.
	 * 
	 * @param deltaCurrent (amps) 0 - 80 amps for max power
	 * @return
	 */
	public int adjustCurrentLimit(final int deltaCurrent) {
		smartCurrentLimit = MathUtil.limit(smartCurrentLimit + deltaCurrent, 0, SMARTCURRENT_MAX);
		// final double secondaryCurrent = smartCurrentLimit * KSecondaryCurrent;

		for (final CANSparkMax c : controllers) {
			// smart current limit
			c.setSmartCurrentLimit(smartCurrentLimit);
		}
		SmartDashboard.putNumber("motorI", smartCurrentLimit);
		System.out.println("***** SmartCurrent*****" + smartCurrentLimit);
		return smartCurrentLimit;
	}

	private void resetControllers() {
		for (final CANSparkMax c : controllers) {
			c.restoreFactoryDefaults(true); // flash them too
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
		Gear currentGear = gearbox.getCurrentGear(); 
		boolean shifting = (currentGear != requestedGear);
		//shifting 
		if (shifting) {
			// do math on new gear to match RPM of new Vel Cmd
			currentGear = requestedGear;   
		}

		// Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
		double k = getFPSperRPM(currentGear);
		double maxSpeed = maxFPS_High;   //getMaxSpeed(currentGear);
		
		// limit vel to max for the gear ratio
		double vcmd = Math.copySign(MathUtil.limit(Math.abs(velFps), 0.0, maxSpeed), velFps);
		double rpm = vcmd / k;

		/**
		 * Rotation controls
		 */
		// Convert to rad/s split between each wheel
		double rps = Math.copySign(MathUtil.limit(Math.abs(rotDps), 0.0, maxDPS), rotDps);
		rps *= (Math.PI / 180.0);
		double vturn_rpm = (rps * WHEEL_AXLE_DIST) / k;

		// add in the commanded speed each wheel
		double vl_rpm = applyDeadZone(rpm + vturn_rpm, RPM_DZ);
		double vr_rpm = -applyDeadZone(rpm - vturn_rpm, RPM_DZ);

		/* debugging
		double v_test = getLeftVel(false);
		double v_err = v_test - vl_rpm;
		*/
		
		if (coastMode) {
			// command doesn't need power, ok to coast, should use PWM and zero power
			// with the controller setup to coast mode by default.
			leftController.set(0.0);
			rightController.set(0.0);
		}
		else {
			// command the velocity to the wheels
			leftController.setReference(vl_rpm);
			rightController.setReference(vr_rpm);
		}

		dDrive.feed();
		
		//shifts the gears if needed
		shiftGears();
	}

	public void arcadeDrive(final double xSpeed, final double zRotation) {
		shiftGears();
		dDrive.arcadeDrive(xSpeed, zRotation, false);
	}

	//both speeds are in ft/s
	public void velocityTankDrive(double leftSpeed, double rightSpeed) {
		// used to handle shifting request
		Gear currentGear = gearbox.getCurrentGear(); 
		boolean shifting = (currentGear != requestedGear);
		//shifting 
		if (shifting) {
			// do math on new gear to match RPM of new Vel Cmd
			currentGear = requestedGear;   
		}

		// Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
		double k = getFPSperRPM(currentGear);
		double maxSpeed = maxFPS_High;   //getMaxSpeed(currentGear);
		
		// limit vel to max for the gear ratio
		double lVcmd = Math.copySign(MathUtil.limit(Math.abs(leftSpeed), 0.0, maxSpeed), leftSpeed);
		double lRpm = lVcmd / k;

		double rVcmd = Math.copySign(MathUtil.limit(Math.abs(rightSpeed), 0.0, maxSpeed), rightSpeed);
		double rRpm = rVcmd / k;

		// add in the commanded speed each wheel
		double vl_rpm = applyDeadZone(lRpm, RPM_DZ);
		double vr_rpm = -applyDeadZone(rRpm, RPM_DZ);

		/* debugging
		double v_test = getLeftVel(false);
		double v_err = v_test - vl_rpm;
		*/
		
		if (coastMode) {
			// command doesn't need power, ok to coast, should use PWM and zero power
			// with the controller setup to coast mode by default.
			leftController.set(0.0);
			rightController.set(0.0);
		}
		else {
			// command the velocity to the wheels
			leftController.setReference(vl_rpm);
			rightController.setReference(vr_rpm);
		}

		dDrive.feed();
		
		//shifts the gears if needed
		shiftGears();
	}

	//Wheel speeds in m/s
	public void velocityTankWheelSpeeds(final double leftWheelSpeed, final double rightWheelSpeed) {
		double leftFps = leftWheelSpeed * 0.3048;//Convert meters to feet
		double rightFps = rightWheelSpeed * 0.3048;
		velocityTankDrive(leftFps, rightFps);
	}

	public void tankDrive(final double leftSpeed, final double rightSpeed) {
		shiftGears();
		dDrive.tankDrive(leftSpeed, rightSpeed, false);
	}

	public Pose2d getPose(){
		return odometry.getPoseMeters();
	}

	public double getHeading() {
		return navX.getAngle() % 360;
	}

	/**
	 * getMaxSpeed based on gear and RPM limits set on motors.
	 * 
	 * @return max speed in ft/s
	 */
	public double getMaxSpeed( Gear gear) {
		return (gear == Gear.HIGH_GEAR) ? maxFPS_High : maxFPS_Low;
	}

	private double getFPSperRPM(Gear gear) {
		return (gear == Gear.HIGH_GEAR) ?  K_high_fps_rpm : K_low_fps_rpm;
	}


	public double getLeftPos() {
		// postion is in units of revs
		double kft_rev = getFPSperRPM(getCurrentGear())*60.0;
		double ft =  (kft_rev) * leftController.getPosition();  
		return ft;
	}

	public double getRightPos() {
		// postion is in units of revs
		double kft_rev = getFPSperRPM(getCurrentGear())*60.0;
		double ft = kft_rev * rightController.getPosition();
		return ft;
	}

	public double getLeftVel(final boolean normalized) {
		double vel =  -leftController.get();           
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel;
	}

	public double getRightVel(final boolean normalized) {
		double vel = rightController.get();
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel;
	}

	public double getAvgVelocity(final boolean normalized) {
		double vel = 0.5*(rightController.get() - leftController.get()); 
		double fps = vel * getFPSperRPM(getCurrentGear());
		vel = (normalized) ? (fps / maxFPS_High) : fps;
		return vel;
	}

	public void resetPosition() {
		rightController.setPosition(0);
		leftController.setPosition(0);
	}


	/**
	 * shiftGears() called in coordination of velocity match
	 */
	void shiftGears() {
		// do nothing if gears match
		if (requestedGear == gearbox.getCurrentGear()) return;
		
		// shift based on requested
		if (requestedGear == Gear.LOW_GEAR) {
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

	/**
	 * Shifter controls encapsulation
	 */
	public void shiftUp() {
		requestedGear = Gear.HIGH_GEAR;
	}

	public void shiftDown() {
		requestedGear = Gear.LOW_GEAR;
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
		 */
		SmartDashboard.putString("Drive Train Default Command", getDefaultCommand().toString());
	}

	/**
	 * VelController puts all the Rev elements into a single class
	 * so it can support SpeedController.  Most of it is just wrapper stuff
	 * but to use the RPM velocity mode, call setReference().
	 * 
	 * Postion will return revs
	 * 
	 */
	class VelController implements SpeedController {
		final CANSparkMax controller;
		final CANPIDController pid;
		final CANEncoder encoder;

		public VelController(final CANSparkMax ctrl) {
			controller = ctrl;
			encoder = controller.getEncoder();
			pid = controller.getPIDController();

			// set PID coefficients
			pid.setP(kP, pidSlot);
			pid.setI(kI, pidSlot);
			pid.setD(kD, pidSlot);
			pid.setIZone(kIz, pidSlot);
			pid.setFF(kFF, pidSlot);
			pid.setOutputRange(kMinOutput, kMaxOutput);
		}

		@Override
		public void pidWrite(final double output) {
			set(output);
		}

		@Override
		public void set(final double speed) {
			controller.set(speed);
		}

		/**
		 * setReference - uses physical units (rpm)
		 * 
		 * @param rpm - revs per minute
		 */
		public void setReference(double rpm) {
			double ffSign = (rpm >= 0) ? -1.0 : 1.0;

			//rpm haz a deadzone so == 0.0 is a fair test.
			if (rpm ==0.0) ffSign = 0.0;
			pid.setReference(rpm, ControlType.kVelocity, pidSlot, ffSign*feedFwd); 
		}

		@Override
		public double get() {
			return encoder.getVelocity();
		}

		public double getPosition() {
			return encoder.getPosition();
		}

		public void setPosition(final double position) {
			encoder.setPosition(position);
		}

		@Override
		public void setInverted(final boolean isInverted) {
			controller.setInverted(isInverted);
		}

		@Override
		public boolean getInverted() {
			return controller.getInverted();
		}

		@Override
		public void disable() {
			set(0.0);
		}

		@Override
		public void stopMotor() {
			set(0.0);
		}
	}

	

	/**
	 *  sleep - eat the exception
	 * @param ms
	 */
	void sleep(final long ms) {
		try {
			TimeUnit.MILLISECONDS.sleep(ms);
		} catch (final InterruptedException e) {
			// don't care
		}
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

}
