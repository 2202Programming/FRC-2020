package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GearShifter;

public class ToggleAutoShift extends InstantCommand {
    private GearShifter shifter;
    private Drivetrain drive;

    public ToggleAutoShift(GearShifter shifter, Drivetrain drive) {
        this.shifter = shifter;
        this.drive = drive;
    }

    @Override
    public void execute() {
        if(shifter.getDefaultCommand() != null && shifter.getDefaultCommand().getName().equals("AutomaticGearShift")) {
            CommandScheduler.getInstance().setDefaultCommand(shifter, new AutomaticGearShift(drive, shifter));
            shifter.setAutoShift(false);
        } else {
            CommandScheduler.getInstance().setDefaultCommand(shifter, new AutomaticGearShift(drive, shifter));
            shifter.setAutoShift(true);
        }
    }
}