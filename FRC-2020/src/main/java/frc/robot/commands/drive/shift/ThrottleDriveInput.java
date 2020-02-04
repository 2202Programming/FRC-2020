package frc.robot.commands.drive.shift;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * An example command. You can replace me with your own command.
 */
public class ThrottleDriveInput implements Command {
    private final double CYCLE_TIME_IN_SECONDS = 0.020;
    private int cycleCount;
    private int maxCycles;
    private double stepValue;
    private double startValue;

    private DriveTrain drive;

    /**
     * 
     * @param rampTime Ramp up time in seconds
     */
    public ThrottleDriveInput(DriveTrain drive, double rampTime, double startValue, double endValue) {
        // Use requires() here to declare subsystem dependencies
        this.drive = drive;
        maxCycles = (int) Math.ceil(rampTime / CYCLE_TIME_IN_SECONDS);
        this.startValue = startValue;
        stepValue = (endValue - startValue) / maxCycles;
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
        drive.arcadeDrive(throttle, turnRate, true);
    }

    @Override
    public boolean isFinished() {
        double leftSpeed = Math.abs(drive.getLeftVel());
        double rightSpeed = Math.abs(drive.getRightVel());
        double curSpeed = (leftSpeed + rightSpeed) / 2.0;
        double shiftSpeed = AutomaticGearShift.DOWNSHIFT_SPEED_LOW * AutomaticGearShift.MAXSPEED_IN_COUNTS_PER_SECOND;
        return cycleCount >= maxCycles || curSpeed < shiftSpeed;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subs = new HashSet<Subsystem>();
        subs.add(drive);
        return subs;
    }
}