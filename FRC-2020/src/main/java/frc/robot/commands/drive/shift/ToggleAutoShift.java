package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
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
        if(shifter.getDefaultCommand().getName().equals("AutomaticGearShift")) {
            shifter.setDefaultCommand(null);
            shifter.setAutoShift(false);
        } else {
            shifter.setDefaultCommand(new AutomaticGearShift(drive, shifter));
            shifter.setAutoShift(true);
        }
    }
}