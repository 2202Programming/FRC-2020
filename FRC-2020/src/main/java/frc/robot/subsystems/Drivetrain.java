package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.*;

public class Drivetrain implements Subsystem {

        // TODO: find actual values for new chassis
        public final double ENCODER_RIGHT_DISTANCE_PER_PULSE = 0.002396219298; // TODO: look into the
                                                                               // getPositionConversionFactor method
        public final double ENCODER_LEFT_DISTANCE_PER_PULSE = 0.002399087014;
        public final int ENCODER_COUNTS_PER_REVOLUTION = 8192;
        public final double WHEEL_RADIUS = 3; //inches
        private final double K_rev_per_ft = 1.0 / (2.0*Math.PI*(WHEEL_RADIUS/12.0)); // rev/feet

        public final double kSamplePeriod = 0.1; // TODO: Find what this is w/ SparkMAX

        private CANSparkMax frontRight = new CANSparkMax(FR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax frontLeft = new CANSparkMax(FL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backRight = new CANSparkMax(BR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax backLeft = new CANSparkMax(BL_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax middleRight = new CANSparkMax(MR_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax middleLeft = new CANSparkMax(ML_SPARKMAX_CANID, CANSparkMaxLowLevel.MotorType.kBrushless);

        private CANEncoder leftEncoder;
        private CANEncoder rightEncoder;
        private CANPIDController leftPidController;
        private CANPIDController rightPidController;

        // PID coefficients ###-move these constants -
        public double kP = 5e-5;
        public double kI = 0.0; //1e-6;
        public double kD = 0;
        public double kIz = 0;
        public double kFF = 0;
        public double kMaxOutput = 1;
        public double kMinOutput = -1;
        public double maxRPM = 5700;

        private DifferentialDrive drive;
        private GearShifter gearbox = null;

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

                // velocity setup
                leftPidController = initVelocityControl(frontLeft);
                rightPidController = initVelocityControl(frontRight);
        }

        public Drivetrain(GearShifter gearbox) 
        {
                this();
                this.gearbox = gearbox;
        }

        public void arcadeDrive(double xSpeed, double rotationSpeed) {
                drive.arcadeDrive(xSpeed, rotationSpeed);
        }

        // vel is ft/s positive forward
        // rotationRate deg/sec
        public void velocityDrive(double vel, double rotationRate) {
                // Spark Max uses RPM for velocity closed loop mode
                // so we need to convert ft/s to RPM command which is dependent 
                // on the gear ratio.

                double rpm = vel*K_rev_per_ft*60.0;
                //adjust for the gearbox setting
                if (null != gearbox) {
                        rpm = rpm*gearbox.getGearRatio();
                }  

                //normally drive.arcadeDrive would be called, but we are using
                //velocity mode so we have to hit the controller directly and re-create the arcade 
                //drive code.
                leftPidController.setReference(rpm, ControlType.kVelocity);
                rightPidController.setReference(rpm, ControlType.kVelocity);

                drive.feed();   //feed the safety watchdog
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

        // Use velocity control for SparkMax - RPM based
        CANPIDController initVelocityControl(CANSparkMax motor) {
                /**
                 * In order to use PID functionality for a controller, a CANPIDController object
                 * is constructed by calling the getPIDController() method on an existing
                 * CANSparkMax object
                 */
                CANPIDController pid = motor.getPIDController();

                // set PID coefficients
                pid.setP(kP);
                pid.setI(kI);
                pid.setD(kD);
                pid.setIZone(kIz);
                pid.setFF(kFF);
                pid.setOutputRange(kMinOutput, kMaxOutput);
                return pid;
        }

}