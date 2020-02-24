package frc.robot.subsystems;

import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.*;
import frc.robot.util.misc.MathUtil;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;

import static frc.robot.Constants.*;

import java.util.concurrent.TimeUnit;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements Logger, DualDrive, VelocityDrive {
	// Current Limits
	private final int SMARTCURRENT_MAX = 60;
	private int smartCurrentLimit = 35; // amps
	// private final double KSecondaryCurrent = 1.40; // set secondary current based
	// on smart current

	// Phyical units deadzone
	private final double RPM_DZ = 2.0;

	// Acceleration limits
	private final double RATE_MAX_SECONDS = 2;
	private double rateLimit = 0.6; // seconds to max speed/power

	// Chasis details
	public final double WHEEL_RADIUS = 4; // inches
	private final double WHEEL_AXLE_DIST = 30.0 / 12.0; // feet
	private final double K_ft_per_rev = (2.0 * Math.PI * WHEEL_RADIUS) / 12.0; // rev/feet
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

	// VelController can use either Velocity mode or dutycycle modes and is wrapper
	// around CANSparkMax. These get setup after factory resets.
	private VelController leftController; 
	private VelController rightController;

	// PID coefficients
	private final double kP = 0.00002;
	private final double kI = 0.000005;
	private final double kD = 0;
	private final double kIz = 0.001;
	private final double kFF = 0.00017;
	private final double kMaxOutput = 1;
	private final double kMinOutput = -1;

	// Calculated based on desired low-gear max ft/s
	private final double maxFPSLow; // max linear speed in ft/s low gear
	private final double maxFPSHigh; //
	private final double maxDPS; // max rotation in deg/sec around Z axis
	private final double maxRPM; // max motor RPM low & high

	private final DifferentialDrive dDrive;
	private GearShifter gearbox;
	private Gear requestedGear; 
	

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear, final double maxFPS_lowGear,
			final double maxDPS) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		this.gearbox = gear;
		this.maxDPS = maxDPS;
		this.requestedGear = gearbox.getCurGear();

		// setup physical units - chassis * gearbox
		K_low_fps_rpm = K_ft_per_rev * gearbox.K_low / 60;
		K_high_fps_rpm = K_ft_per_rev * gearbox.K_high / 60;
		// compute max RPM for motors, use same for low and high gear
		maxRPM = (maxFPS_lowGear / K_low_fps_rpm); // [60s/m]*[ft/s]/[ft/rev] = rev/min
		maxFPSLow = (maxRPM * K_low_fps_rpm); // max speed in low gear
		maxFPSHigh = (maxRPM * K_high_fps_rpm); // max speed in high gear

		// setup SparkMax controllers, sets left and right masters
		configureControllers();

		// zero adjust will set the default limits
		adjustAccelerationLimit(0.0);
		adjustCurrentLimit(0);

		dDrive = new DifferentialDrive(leftController, rightController);
		dDrive.setSafetyEnabled(false);
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
		Gear currentGear = gearbox.getCurGear(); 
		boolean shifting = (currentGear != requestedGear);
		//TODO: finish shifting logic

		// Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
		double k = K_low_fps_rpm;
		double maxSpeed = maxFPSLow;

		if (currentGear == Gear.HIGH_GEAR) {
			k = K_high_fps_rpm;
			maxSpeed = maxFPSHigh;
		}
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

		// command the velocity to the wheels

		leftController.setReference(vl_rpm);
		rightController.setReference(vr_rpm);
		dDrive.feed();
	}

	public void arcadeDrive(final double xSpeed, final double zRotation) {
		dDrive.arcadeDrive(xSpeed, zRotation, false);
	}

	public void tankDrive(final double leftSpeed, final double rightSpeed) {
		dDrive.tankDrive(leftSpeed, rightSpeed, false);
	}

	/**
	 * getMaxSpee based on current gear and RPM limits set on motors.
	 * 
	 * @return max speed in ft/s
	 */
	public double getMaxSpeed() {
		return (gearbox.getCurGear() == Gear.HIGH_GEAR) ? maxFPSHigh : maxFPSLow;
	}

	public double getLeftPos() {
		return K_ft_per_rev * leftController.getPosition();
	}

	public double getLeftVel(final boolean normalized) {
		double vel = leftController.get();
		vel = (normalized) ? (vel / maxRPM) : vel;
		return vel;
	}

	public double getRightPos() {
		return K_ft_per_rev * rightController.getPosition();

	}

	public double getRightVel(final boolean normalized) {
		double vel = rightController.get();
		vel = (normalized) ? (vel / maxRPM) : vel;
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
		if (requestedGear == gearbox.getCurGear()) return;
		
		// shift based on requested
		if (requestedGear == Gear.LOW_GEAR) {
			gearbox.shiftDown();
		} 
		else {
			gearbox.shiftUp();
		}
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
		return gearbox.getCurGear();
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
			pid.setP(kP);
			pid.setI(kI);
			pid.setD(kD);
			pid.setIZone(kIz);
			pid.setFF(kFF);
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
			pid.setReference(rpm, ControlType.kVelocity);
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
