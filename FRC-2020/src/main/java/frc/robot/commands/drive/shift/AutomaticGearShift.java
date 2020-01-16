package frc.robot.commands.drive.shift;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;

public class AutomaticGearShift implements Command {
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
    private Drivetrain drive;

    public AutomaticGearShift(Drivetrain drive, GearShifter shifter) {
        this.drive = drive;
        this.shifter = shifter;
    }

    public void execute() {
        Gear curGear = shifter.getCurGear();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}