package frc.robot.commands.drive.shift;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GearShifter;

public class ToggleAutoShift implements Command {
    private GearShifter shifter;
    private DriveTrain drive;

    public ToggleAutoShift(GearShifter shifter, DriveTrain drive) {
        this.shifter = shifter;
        this.drive = drive;
    }

    @Override
    public void execute() {
        if(CommandScheduler.getInstance().getDefaultCommand(shifter) != null && CommandScheduler.getInstance().getDefaultCommand(shifter).getName().equals("AutomaticGearShift")) {
            shifter.setAutoShift(false);
            CommandScheduler scheduler = CommandScheduler.getInstance();
            scheduler.setDefaultCommand(shifter, null);
        } else {
            CommandScheduler.getInstance().setDefaultCommand(shifter, new AutomaticGearShift(drive, shifter));
            shifter.setAutoShift(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subs = new HashSet<Subsystem>();
        subs.add(shifter);
        return subs;
    }
}