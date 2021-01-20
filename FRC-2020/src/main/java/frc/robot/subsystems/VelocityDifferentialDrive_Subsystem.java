package frc.robot.subsystems;

import static frc.robot.Constants.RobotPhysical.WheelAxelDistance;
import static frc.robot.Constants.RobotPhysical.WheelDiameter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.DualDrive;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;
import frc.robot.util.misc.MathUtil;
import frc.robot.util.misc.PIDFController;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements Logger, DualDrive, VelocityDrive, Shifter {
  
	// Current Limits
	private int smartCurrentLimit = DriveTrain.smartCurrentLimit; // amps
	// private final double KSecondaryCurrent = 1.40; // set secondary current based on smart current

	// Phyical units deadzone
	private final double RPM_DZ = 2.0;

	// Acceleration limits
	private final double RATE_MAX_SECONDS = 2;
	private double rateLimit = 0.9; // seconds to max speed/power

	// Chasis details
	private final double WHEEL_AXLE_DIST = WheelAxelDistance / 12.0; // feet
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
	VelController leftController; 
	VelController rightController;
	
	// likely not needed because it is so low to get robot to move.
	final double ARBIT_FEEDFWD_MAX_VOLT = 0.0;   //0.15 was enough to move Alfred.
	private double arbFeedFwd = 0.0;

	// Calculated based on desired low-gear max ft/s
	private final double maxFPS_High;  // Assigned
	private final double maxFPS_Low;   // using HIGH gear RPM
	private final double maxDPS;       // max rotation in deg/sec around Z axis
	//private final double maxRPM_Low;   // max motor RPM low & high
	private final double maxRPM_High;  // max motor RPM low & high

	final DifferentialDrive dDrive;
	GearShifter gearbox;
	Gear requestedGear; 
	boolean coastMode = false;
	
	//measurements, taken in periodic()
	double velLeft=0.0;   //fps, positive moves robot forward
	double velRight=0.0;  //fps, positive moves robot forward
	Gear currentGear;

	public VelocityDifferentialDrive_Subsystem(final GearShifter gear) {
		// save scaling factors, they are required to use SparkMax in Vel mode
		gearbox = gear;
		maxDPS = DriveTrain.maxRotDPS;
		requestedGear = gearbox.getCurrentGear();
		maxFPS_High = DriveTrain.maxFPS;

		// setup physical units - chassis * gearbox  (rev per minute)
		K_low_fps_rpm = K_ft_per_rev * gearbox.K_low / 60;    //rpm/60 rps
		K_high_fps_rpm = K_ft_per_rev * gearbox.K_high / 60;
    
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

		// zero adjust will set the default limits
		adjustAccelerationLimit(0.0);
		adjustCurrentLimit(0);

		dDrive = new DifferentialDrive(leftController, rightController);
		dDrive.setSafetyEnabled(DriveTrain.safetyEnabled);
	}

	/**
	 * hides some of the ugly setups like delays between programming.
	 */
	void configureControllers() {
		// factory reset
		resetControllers();

		// velocity setup - using RPM speed controller, sets pid
    leftController = new VelController(backLeft, DriveTrain.pidValues);
    leftController.setEncoderInverted(DriveTrain.leftEncoderInverted);
		rightController = new VelController(backRight, DriveTrain.pidValues);
    rightController.setEncoderInverted(DriveTrain.rightEncoderInverted);

		// Have motors follow to use Differential Drive
		middleRight.follow(backRight);   // right side
		frontRight.follow(backRight);
		middleLeft.follow(backLeft);     // left side
		frontLeft.follow(backLeft);

		// zero adjust will set the default limits for accel and currents
		adjustAccelerationLimit(0.0);
		adjustCurrentLimit(0);

		// burn the default value in case of brown-out
		saveControllers();
	}


	@Override
	public void periodic() {
		// read required sensor and system values for the frame
		velLeft = getLeftVel(false);
		velRight = getRightVel(false);
		currentGear = gearbox.getCurrentGear(); 
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

		SmartDashboard.putNumber("DT/limits/arbFF", arbFeedFwd);
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
		rateLimit = MathUtil.limit((rateLimit + deltaRate), 0.0, RATE_MAX_SECONDS);
		// Just set the ramp limit on the masters
		leftController.controller.setOpenLoopRampRate(rateLimit);
		rightController.controller.setOpenLoopRampRate(rateLimit);

		// Use same rate limit on closed loop too
		leftController.controller.setClosedLoopRampRate(rateLimit);
		rightController.controller.setClosedLoopRampRate(rateLimit);

		SmartDashboard.putNumber("DT/limits/acelrate", rateLimit);
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

	public void tankDrive(final double leftSpeed, final double rightSpeed) {
		shiftGears();
		dDrive.tankDrive(leftSpeed, rightSpeed, false);
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
		double ft =  (-kft_rev) * leftController.getPosition();  
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

	/** Shifter is implemented by this class */
	public Shifter getShifter() {
		return this;
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
		 * SmartDashboard.putString("DT Command", getDefaultCommand().toString());
		 * SmartDashboard.putString("Current Gear", gearbox.getCurrentGear().toString());
		*/
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
	
		private final double kMaxOutput = 1;
		private final double kMinOutput = -1;
		private final int pidSlot = 0;

		public VelController(final CANSparkMax ctrl, PIDFController pidf) {
			controller = ctrl;
			encoder = controller.getEncoder();
			pid = controller.getPIDController();

			// set PID coefficients
			pid.setP(pidf.getP(), pidSlot);
			pid.setI(pidf.getI(), pidSlot);
			pid.setD(pidf.getD(), pidSlot);
			pid.setIZone(0.0, pidSlot);
			pid.setFF(pidf.getF(), pidSlot);
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
			double arbff = (rpm >= 0) ? arbFeedFwd : - arbFeedFwd;

			//rpm haz a deadzone so == 0.0 is a fair test.
			if (rpm ==0.0) arbff = 0.0;
			pid.setReference(rpm, ControlType.kVelocity, pidSlot, arbff); 
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

    /**
     * Controller direction can be inverted.  This is useful so both sides
     * of the drivetrain get positive RPM to move forward.
     * 
     */
		@Override
		public void setInverted(final boolean isInverted) {
			controller.setInverted(isInverted);
		}

		@Override
		public boolean getInverted() {
			return controller.getInverted();
		}

    /**
     *    setEncoderInverted(invert)
     *    
     * @param invert  true inverts the encoder sign
     */
    public void setEncoderInverted(boolean invert) {
      encoder.setInverted(invert);
    }

    public boolean getEncoderInverted() {
      return encoder.getInverted();
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
		layout.addNumber("DT/Vel/left", () -> velLeft).withSize(2, 2);
		layout.addNumber("DT/Vel/right", () -> velRight);
		layout.addString("DT/gear", () -> gearbox.getCurrentGear().toString());
		
	}

}
