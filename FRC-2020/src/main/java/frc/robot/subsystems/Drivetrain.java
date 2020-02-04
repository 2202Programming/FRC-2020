package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class DriveTrain implements Subsystem {

	// TODO: find actual values for new chassis
	public final double ENCODER_RIGHT_DISTANCE_PER_PULSE = 0.005;
	public final double ENCODER_LEFT_DISTANCE_PER_PULSE = 0.005;
	public final int ENCODER_COUNTS_PER_REVOLUTION = 4096;
	public final double WHEEL_RADIUS = 3;

    public final double kSamplePeriod = 0.1; // TODO: Find what this is w/ SparkMAX
    
    //TODO: fina actual values for bot
    public static final double trackWidthMeters = 0.69;
    public static final double maxSpeed = 3; // m/s
    public static final double maxAccel = 3; // m/s/s

	private CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);

	private CANEncoder leftEncoder;
	private CANEncoder rightEncoder;

    private DifferentialDriveOdometry odometry;
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

	private DifferentialDrive drive;

	public DriveTrain() {
		// Have motors follow to use Differential Drive
		middleRight.follow(frontRight);
		middleLeft.follow(frontLeft);
		backRight.follow(frontRight);
		backLeft.follow(frontLeft);
		drive = new DifferentialDrive(frontLeft, frontRight);

		odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

		// Encoders
		leftEncoder = frontLeft.getEncoder();
		rightEncoder = frontRight.getEncoder();

	}

	public void arcadeDrive(double xSpeed, double rotationSpeed, boolean squareInputs) {
		drive.arcadeDrive(xSpeed, -rotationSpeed, squareInputs);
	}

	public void tankDrive(double speedLeft, double speedRight, boolean squareInputs)
	{
		drive.tankDrive(speedLeft, speedRight, squareInputs);
	}

	//getPosition() for some reason isn't returning raw counts but instead inches
	public double getLeftPos() {
		return leftEncoder.getPosition(); //* ENCODER_LEFT_DISTANCE_PER_PULSE;
	}

	public double getLeftVel() {
		return leftEncoder.getVelocity() * kSamplePeriod * ENCODER_LEFT_DISTANCE_PER_PULSE;
	}

	public void resetLeftEncoder() {
		leftEncoder.setPosition(0);
	}

	public double getRightPos() {
		return rightEncoder.getPosition(); //* ENCODER_RIGHT_DISTANCE_PER_PULSE;
	}

	public double getRightVel() {
		return rightEncoder.getVelocity() * kSamplePeriod * ENCODER_RIGHT_DISTANCE_PER_PULSE;
	}

	public void resetRightEncoder() {
		rightEncoder.setPosition(0);
    }

    public void resetEncoders() {
        resetRightEncoder();
        resetLeftEncoder();
    }
    
    public double getHeading() {
        return 0.0; //TODO: Figure out how we're calculating heading
    }

    public void resetHeading() {
         //TODO: Figure out how we're calculating heading
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

	public void log() {
		SmartDashboard.putNumber("Right Velocity", getRightVel());
		SmartDashboard.putNumber("Left Velocity", getLeftVel());
		SmartDashboard.putNumber("Right Position", getRightPos());
		SmartDashboard.putNumber("Left Position", getLeftPos());
	}

}