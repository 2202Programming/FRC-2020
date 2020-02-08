package frc.robot.subsystems;

import frc.robot.subsystems.ifx.*;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import static frc.robot.Constants.*;

public class VelocityDifferentialDrive_Subsystem extends SubsystemBase implements ArcadeDrive {

        private final static double MAXRPM = 1000.0;
        private final static double MAXDPS = 5.0;

        public final double WHEEL_RADIUS = 3; // inches
        private final double K_rev_per_ft = 1.0 / (2.0 * Math.PI * (WHEEL_RADIUS / 12.0)); // rev/feet

        // CANSpark Max will be used 3 per side, 2 folowing the leader
        private final CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private final CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);

        private final VelController leftPidController;
        private final VelController rightPidController;

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

                // Have motors follow to use Differential Drive
                middleRight.follow(frontRight);
                middleLeft.follow(frontLeft);
                backRight.follow(frontRight);
                backLeft.follow(frontLeft);
                gearbox = gear;

                // velocity setup - using RPM speed controller
                leftPidController = initVelocityControl(frontLeft);
                rightPidController = initVelocityControl(frontRight);

                dDrive = new DifferentialDrive(leftPidController, rightPidController);
                dDrive.setSafetyEnabled(false);
        }

        public VelocityDifferentialDrive_Subsystem(final GearShifter gear) {
                this(gear, MAXRPM, MAXDPS);
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
                dDrive.arcadeDrive(xSpeed, zRotation);
        }

        public void tankDrive(double leftSpeed, double rightSpeed) {
                dDrive.tankDrive(leftSpeed, rightSpeed, false);
        }

        public double getLeftPos() {
                return leftPidController.getPosition();
        }

        public double getLeftVel(boolean normalized) {
                double vel = leftPidController.get();
                vel = (normalized) ? (vel / maxRPM) : vel;
                return vel;
        }

        public double getRightPos() {
                return rightPidController.getPosition();
        }

        public double getRightVel(boolean normalized) {
                double vel = rightPidController.get();
                vel = (normalized) ? (vel / maxRPM) : vel;
                return vel;
        }

        public void resetPositon() {
                rightPidController.setPosition(0);
                leftPidController.setPosition(0);
        }

        public void log() {
                SmartDashboard.putNumber("Right Velocity", getRightVel(false));
                SmartDashboard.putNumber("Left Velocity", getLeftVel(false));
                SmartDashboard.putNumber("Right Position", getRightPos());
                SmartDashboard.putNumber("Left Position", getLeftPos());
        }

        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        }

        /**
         * Returns the current wheel speeds of the robot.
         *
         * @return The current wheel speeds.
         */
        public DifferentialDriveWheelSpeeds getWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(leftPidController.get(), rightPidController.get());
        }

        /**
         * Controls the left and right sides of the drive directly with voltages.
         *
         * @param leftVolts  the commanded left output
         * @param rightVolts the commanded right output
         */
        public void tankDriveVolts(double leftVolts, double rightVolts) {
                frontLeft.setVoltage(leftVolts);
                frontRight.setVoltage(-rightVolts);
                dDrive.feed();
        }

        /**
         * Resets the drive encoders to currently read a position of 0.
         */
        public void resetEncoders() {
                leftPidController.setPosition(0);
                rightPidController.setPosition(0);
        }

        /**
         * Gets the average distance of the two encoders.
         *
         * @return the average of the two encoder readings
         */
        public double getAverageEncoderDistance() {
                return (leftPidController.getPosition() + rightPidController.getPosition()) / 2.0;
        }

        /**
         * Sets the max output of the drive. Useful for scaling the drive to drive more
         * slowly.
         *
         * @param maxOutput the maximum output to which the drive will be constrained
         */
        public void setMaxOutput(double maxOutput) {
                dDrive.setMaxOutput(maxOutput);
        }

        /**
         * Zeroes the heading of the robot.
         */
        public void zeroHeading() {
                m_gyro.reset();
        }

        /**
         * Returns the heading of the robot.
         *
         * @return the robot's heading in degrees, from -180 to 180
         */
        public double getHeading() {
                return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }

        /**
         * Returns the turn rate of the robot.
         *
         * @return The turn rate of the robot, in degrees per second
         */
        public double getTurnRate() {
                return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }

        // Use velocity control for SparkMax - RPM based
        VelController initVelocityControl(final CANSparkMax motor) {
                /**
                 * In order to use PID functionality for a controller, a CANPIDController object
                 * is constructed by calling the getPIDController() method on an existing
                 * CANSparkMax object
                 */
                final CANPIDController pid = motor.getPIDController();
                final VelController velController = new VelController(motor); // use our SpeedController
                // set PID coefficients
                pid.setP(kP);
                pid.setI(kI);
                pid.setD(kD);
                pid.setIZone(kIz);
                pid.setFF(kFF);
                pid.setOutputRange(kMinOutput, kMaxOutput);
                return velController;
        }

        public class VelController implements SpeedController {
                final CANSparkMax controller;
                final CANPIDController pid;
                final CANEncoder encoder;

                public VelController(final CANSparkMax controller) {
                        this.controller = controller;
                        this.encoder = controller.getEncoder();
                        this.pid = controller.getPIDController();
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
                        final double rpm = speed * maxRPM;
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
}
