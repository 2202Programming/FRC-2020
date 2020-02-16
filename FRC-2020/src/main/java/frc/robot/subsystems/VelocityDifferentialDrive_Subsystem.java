package frc.robot.subsystems;

import frc.robot.subsystems.ifx.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import static frc.robot.Constants.*;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements DualDrive, ArcadeDrive, TankDrive {

	private final static double MAXRPM = 15000.0;
	private final static double MAXDPS = 5.0;

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

	private final VelController leftController;
	private final VelController rightController;

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

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear, final double maxRPM, final double maxDPS) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		this.maxRPM = maxRPM;
		this.maxDPS = maxDPS;

		setCoastMode();

		// Have motors follow to use Differential Drive
		middleRight.follow(frontRight);
		middleLeft.follow(frontLeft);
		backRight.follow(frontRight);
		backLeft.follow(frontLeft);
		gearbox = gear;

		// velocity setup - using RPM speed controller
		leftController = new VelController(frontLeft);
		rightController = new VelController(frontRight);

		setVelocityMode(false);

		dDrive = new DifferentialDrive(leftController, rightController);
		dDrive.setSafetyEnabled(false);
	}

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear) {
		this(gear, MAXRPM, MAXDPS);
	}

	void setCoastMode() {
		frontRight.setIdleMode(IdleMode.kCoast);
		middleRight.setIdleMode(IdleMode.kCoast);
		backRight.setIdleMode(IdleMode.kCoast);

		frontLeft.setIdleMode(IdleMode.kCoast);
		middleLeft.setIdleMode(IdleMode.kCoast);
		backLeft.setIdleMode(IdleMode.kCoast);
	}

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
		SmartDashboard.putNumber("Right Velocity", getRightVel(false));
		SmartDashboard.putNumber("Left Velocity", getLeftVel(false));
		SmartDashboard.putNumber("Right Position", getRightPos());
		SmartDashboard.putNumber("Left Position", getLeftPos());
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

}
