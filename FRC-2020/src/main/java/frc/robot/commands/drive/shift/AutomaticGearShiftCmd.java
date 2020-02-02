package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.ArcadeDrive;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.ifx.DriverControls;

public class AutomaticGearShiftCmd extends CommandBase {
    public static final double MAXSPEED_IN_COUNTS_PER_SECOND = 10000; // TODO: Find real values for these constants
    public static final double UPSHIFT_SPEED_LOW = 0.3;
    public static final double UPSHIFT_SPEED_HIGH = 0.3;
    public static final double UPSHIFT_THROTTLE_LOW = 0.3;
    public static final double UPSHIFT_THROTTLE_HIGH = 0.6;
    public static final double DOWNSHIFT_SPEED_LOW = 0.03;
    public static final double DOWNSHIFT_SPEED_HIGH = 0.03;
    public static final double DOWNSHIFT_THROTTLE_LOW = 0.3;
    public static final double DOWNSHIFT_THROTTLE_HIGH = 0.6;
    public static final double DEADZONE = 0.02;
    public static final double TURNING_DEADZONE = 500;

    private final double MAX_OUTPUT = 1.0;
    private final double RIGHT_SIDE_INVERT_MULTIPLIER = -1.0;

    private GearShifter shifter;
    private ArcadeDrive drive;
    private DriverControls dc;

    public AutomaticGearShiftCmd(DriverControls dc, ArcadeDrive drive, GearShifter shifter) {
        this.drive = drive;
        this.shifter = shifter;
        this.dc = dc;            //don't register dc, it runs on periodic

        addRequirements(drive, shifter);
    }

    public void execute() {
        Gear curGear = shifter.getCurGear();
        double leftSpeed = Math.abs(drive.getLeftVel(true));
        double rightSpeed = Math.abs(drive.getRightVel(true));
        double curSpeed = (leftSpeed + rightSpeed) / 2;
        double shiftSpeed = getShiftSpeed(shifter.getCurGear(), getThrottle(true));

        if (Math.abs(leftSpeed - rightSpeed) > TURNING_DEADZONE) {
            // If outside of turning deadzone don't shift
            return;
        }

        if (curGear == Gear.LOW_GEAR) {
            // Disable automatic upshift (aka only auto downshift)
            // if (curSpeed > shiftSpeed) {
            // new AutomaticUpShiftCommand().start();
            // }
        } else {
            if (curSpeed < shiftSpeed) {
                shifter.shiftDown();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Taken from DifferentialDrive arcadeDrive method Gets the minimum throttle
     * between the two sides of the robot
     * 
     * @param squareInputs Whether you are using squared inputs
     * @return The minimum throttle
     */
    private double getThrottle(boolean squareInputs) {
        double xSpeed = dc.getVelocity();
        double zRotation = dc.getRotation();

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        double leftThrottle = Math.abs(limit(leftMotorOutput) * MAX_OUTPUT);
        double rightThrottle = Math.abs(limit(rightMotorOutput) * MAX_OUTPUT * RIGHT_SIDE_INVERT_MULTIPLIER);

        return Math.min(leftThrottle, rightThrottle);
    }
    
    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    private double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Calculates the shift speed threshold based on the current gear and throttle
     * 
     * @param curGear     The current gear
     * @param curThrottle The current minimum throttle
     * @return The shift speed threshold
     */
    private double getShiftSpeed(Gear curGear, double curThrottle) {
        if (curGear == Gear.LOW_GEAR) {
            if (curThrottle < UPSHIFT_THROTTLE_LOW) {
                return MAXSPEED_IN_COUNTS_PER_SECOND * UPSHIFT_SPEED_LOW;
            } else if (curThrottle < UPSHIFT_SPEED_HIGH) {
                double slope = (UPSHIFT_SPEED_HIGH - UPSHIFT_SPEED_LOW)
                        / (UPSHIFT_THROTTLE_HIGH - UPSHIFT_THROTTLE_LOW);
                double maxSpeedProportion = UPSHIFT_SPEED_LOW + slope * (curThrottle - UPSHIFT_THROTTLE_LOW);
                return maxSpeedProportion * MAXSPEED_IN_COUNTS_PER_SECOND;
            } else {
                return MAXSPEED_IN_COUNTS_PER_SECOND * UPSHIFT_SPEED_HIGH;
            }
        } else {
            if (curThrottle < DOWNSHIFT_THROTTLE_LOW) {
                return MAXSPEED_IN_COUNTS_PER_SECOND * DOWNSHIFT_SPEED_LOW;
            } else if (curThrottle < DOWNSHIFT_SPEED_HIGH) {
                double slope = (DOWNSHIFT_SPEED_HIGH - DOWNSHIFT_SPEED_LOW)
                        / (DOWNSHIFT_THROTTLE_HIGH - DOWNSHIFT_THROTTLE_LOW);
                double maxSpeedProportion = DOWNSHIFT_SPEED_LOW + slope * (curThrottle - DOWNSHIFT_THROTTLE_LOW);
                return maxSpeedProportion * MAXSPEED_IN_COUNTS_PER_SECOND;
            } else {
                return MAXSPEED_IN_COUNTS_PER_SECOND * DOWNSHIFT_SPEED_HIGH;
            }
        }
    }

}