package frc.robot.subsystems;

import frc.robot.subsystems.ifx.*;
import frc.robot.util.misc.MathUtil;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;

import static frc.robot.Constants.*;

import java.util.concurrent.TimeUnit;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements Logger, DualDrive {
	// Current Limits
	private final int SMARTCURRENT_MAX = 60;
	private int smartCurrentLimit = 50; // amps
	private final double KSecondaryCurrent = 1.40; // set secondary current based on smart current

	// Acceleration limits
	private final double RATE_MAX_SECONDS = 2;
	private double rateLimit = 0.2; // seconds to max speed/power

	// Chasis details
	public final double WHEEL_RADIUS = 4; // inches
	private final double K_ft_per_rev = (2.0 * Math.PI * WHEEL_RADIUS) / 12.0; // rev/feet
	private final double K_rev_per_ft = 1.0 / K_ft_per_rev;

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
	// around CANSparkMax
	private VelController leftController;
	private VelController rightController;

	// PID coefficients TODO: move these constants
	private double kP = 5e-5;
	private double kI = 0.0; // 1e-6;
	private double kD = 0;
	private double kIz = 0;
	private double kFF = 0;
	private double kMaxOutput = 1;
	private double kMinOutput = -1;
	private double maxRPM;
	private double maxDPS;

	private final DifferentialDrive dDrive;
	private GearShifter gearbox = null;
	CANError err;

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear, final double maxRPM, final double maxDPS) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		this.maxRPM = maxRPM;
		this.maxDPS = maxDPS;
		this.gearbox = gear;

		// setup SparkMax controllers, sets left and right masters
		configureControllers();
		setVelocityMode(false);

		dDrive = new DifferentialDrive(leftController, rightController);
		dDrive.setSafetyEnabled(false);
	}

	/**
	 * hides some of the ugly setups like delays between programming.
	 */
	void configureControllers() {
		CANSparkMax rMaster = backRight;
		CANSparkMax lMaster = backLeft;

		// velocity setup - using RPM speed controller
		this.leftController = new VelController(lMaster);
		this.rightController = new VelController(rMaster);
		
		// factory reset
		resetControllers();

		// Have motors follow to use Differential Drive
		err = middleRight.follow(rMaster);
		//sleep(2); // hack to ensure timing
		err = frontRight.follow(rMaster);
		//sleep(2);
		err = middleLeft.follow(lMaster);
		//sleep(2); // hack to ensure timing
		err = frontLeft.follow(lMaster);

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
	public double adjustAccelerationLimit(double deltaRate) {
		rateLimit = MathUtil.limit((rateLimit + deltaRate), 0.0, RATE_MAX_SECONDS);
		// Just set the ramp limit on the masters
		leftController.controller.setOpenLoopRampRate(rateLimit);
		sleep(1);
		rightController.controller.setOpenLoopRampRate(rateLimit);
		SmartDashboard.putNumber("motorRate", rateLimit);
		return rateLimit;
	}

	/**
	 * Change the default smart current limits for drive motors Also adjusts
	 * secondary to smart limit +20%
	 * 
	 * @param deltaCurrent (amps) 0 - 80 amps for max power
	 * @return
	 */
	public int adjustCurrentLimit(int deltaCurrent) {
		smartCurrentLimit += deltaCurrent;
		smartCurrentLimit = MathUtil.limit(smartCurrentLimit, 0, SMARTCURRENT_MAX);
		double secondaryCurrent = smartCurrentLimit * KSecondaryCurrent;

		for (CANSparkMax c : controllers) {
			// smart current limit
			c.setSmartCurrentLimit(smartCurrentLimit);

			// Set the secondary current based on the smartCurrent
			// c.setSecondaryCurrentLimit(secondaryCurrent);
		}
		SmartDashboard.putNumber("motorI", smartCurrentLimit);
		System.out.println("***** SmartCurrent*****" + smartCurrentLimit);
		return smartCurrentLimit;
	}

	private void resetControllers() {
		for (CANSparkMax c : controllers) {
			c.restoreFactoryDefaults(false);
			//sleep(2);
		}
	}

	private void saveControllers() {
		for (CANSparkMax c : controllers) {
			c.burnFlash();
			//sleep(2);
		}
	}

	/**
	 * Enables using physical units for driving.
	 * 
	 * @param useVelocity
	 */
	public void setVelocityMode(boolean useVelocity) {
		leftController.setVelocityMode(useVelocity);
		rightController.setVelocityMode(useVelocity);
	}

	// vel is ft/s positive forward
	// rotationRate deg/sec
	public void velocityArcadeDrive(final double velFps, final double rotDps) {
		// Spark Max uses RPM for velocity closed loop mode
		// so we need to convert ft/s to RPM command which is dependent
		// on the gear ratio.
		double rpm = velFps * K_rev_per_ft * 60.0;

		// adjust for the gearbox setting
		if (null != gearbox) {
			rpm = rpm * gearbox.getGearRatio();
		}
		/**
		 * arcadeDrive require normalized units because it clamps the speed/rotation
		 * values. This converts to normalized by using our spec'd max speed/rotations.
		 */
		final double xSpeed = rpm / maxRPM;
		final double zRotation = rotDps / maxDPS;

		// call arcadeDrive with the silly normalized speeds
		dDrive.arcadeDrive(xSpeed, zRotation, false);
	}

	// TODO: velocityTankDrive()

	public void arcadeDrive(double xSpeed, double zRotation) {
		dDrive.arcadeDrive(xSpeed, zRotation, false);
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		dDrive.tankDrive(leftSpeed, rightSpeed, false);
	}

	public double getLeftPos() {
		return K_ft_per_rev * leftController.getPosition();
	}

	public double getLeftVel(boolean normalized) {
		double vel = leftController.get();
		vel = (normalized) ? (vel / maxRPM) : vel;
		return vel;
	}

	public double getRightPos() {
		return K_ft_per_rev * rightController.getPosition();

	}

	public double getRightVel(boolean normalized) {
		double vel = rightController.get();
		vel = (normalized) ? (vel / maxRPM) : vel;
		return vel;
	}

	public void resetPosition() {
		rightController.setPosition(0);
		leftController.setPosition(0);
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

	public class VelController implements SpeedController {
		final CANSparkMax controller;
		final CANPIDController pid;
		final CANEncoder encoder;
		boolean velocityMode = true;

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

		public void setVelocityMode(boolean useVelocity) {
			velocityMode = useVelocity;
		}

		public boolean getVelocityMode() {
			return velocityMode;
		}

		@Override
		public void pidWrite(final double output) {
			set(output);
		}

		/**
		 * This is the critical change, using controlType.kVelocity Everything else is
		 * mostly a pass through
		 */
		@Override
		public void set(final double speed) {
			if (velocityMode) {
				final double rpm = speed * maxRPM;
				pid.setReference(rpm, ControlType.kVelocity);
			} else {
				controller.set(speed);
				// pid.setReference(speed, ControlType.kDutyCycle);
			}
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

	void sleep(long ms) {
		try {
			TimeUnit.MILLISECONDS.sleep(ms);
		} catch (InterruptedException e) {
			// don't care
		}
	}

}
