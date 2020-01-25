package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.*;

public class Drivetrain implements Subsystem {

	// TODO: find actual values for new chassis
	public final double ENCODER_RIGHT_DISTANCE_PER_PULSE = 0.002396219298; // TODO: look into the
																			// getPositionConversionFactor method
	public final double ENCODER_LEFT_DISTANCE_PER_PULSE = 0.002399087014;
	public final int ENCODER_COUNTS_PER_REVOLUTION = 8192;
	public final double WHEEL_RADIUS = 3;

	public final double kSamplePeriod = 0.1; // TODO: Find what this is w/ SparkMAX

	private CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);

	private CANEncoder leftEncoder;
	private CANEncoder rightEncoder;

	private DifferentialDrive drive;

	public Drivetrain() {
		// Have motors follow to use Differential Drive
		middleRight.follow(frontRight);
		middleLeft.follow(frontLeft);
		backRight.follow(frontRight);
		backLeft.follow(frontLeft);
		drive = new DifferentialDrive(frontLeft, frontRight);

		// Encoders
		leftEncoder = frontLeft.getEncoder();
		rightEncoder = frontRight.getEncoder();

	}

	public void arcadeDrive(double xSpeed, double rotationSpeed, boolean squareInputs) {
		drive.arcadeDrive(xSpeed, -rotationSpeed, squareInputs);
	}

	public double getLeftPos() {
		return leftEncoder.getPosition() * ENCODER_LEFT_DISTANCE_PER_PULSE;
	}

	public double getLeftVel() {
		return leftEncoder.getVelocity() * kSamplePeriod * ENCODER_LEFT_DISTANCE_PER_PULSE;
	}

	public void resetLeftEncoder() {
		leftEncoder.setPosition(0);
	}

	public double getRightPos() {
		return rightEncoder.getPosition() * ENCODER_RIGHT_DISTANCE_PER_PULSE;
	}

	public double getRightVel() {
		return rightEncoder.getVelocity() * kSamplePeriod * ENCODER_RIGHT_DISTANCE_PER_PULSE;
	}

	public void resetRightEncoder() {
		rightEncoder.setPosition(0);
	}

}