package frc.robot.commands.drive.shift;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ifx.ArcadeDrive;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleDriveInputCmd extends CommandBase {
    private int cycleCount;
    private int maxCycles;
    private double stepValue;
    private double startValue;

    private ArcadeDrive drive;

    /**
     * 
     * @param rampTime Ramp up time in seconds
     */
    public ThrottleDriveInputCmd(ArcadeDrive drive, double rampTime, double startValue, double endValue) {
        // Use requires() here to declare subsystem dependencies
        this.drive = drive;
        maxCycles = (int) Math.ceil(rampTime / Constants.DT);
        this.startValue = startValue;
        stepValue = (endValue - startValue) / maxCycles;

        addRequirements(drive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        cycleCount = 0;
        execute();
    }

    // Read Controller Input from two joysticks.
    // Left joystick controls the left motors and the right joystick controls the
    // right motors
    @Override
    public void execute() {
        double throttle = RobotContainer.driver.getY(Hand.kLeft) * (startValue + stepValue * cycleCount);
        double turnRate = RobotContainer.driver.getX(Hand.kRight) * (startValue + stepValue * cycleCount);
        cycleCount++;
        drive.arcadeDrive(throttle, turnRate);
    }

    @Override
    public boolean isFinished() {
        double leftSpeed = Math.abs(drive.getLeftVel(true));
        double rightSpeed = Math.abs(drive.getRightVel(true));
        double curSpeed = (leftSpeed + rightSpeed) / 2.0;
        double shiftSpeed = AutomaticGearShiftCmd.DOWNSHIFT_SPEED_LOW * AutomaticGearShiftCmd.MAXSPEED_IN_COUNTS_PER_SECOND;
        return cycleCount >= maxCycles || curSpeed < shiftSpeed;
    }
}