package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ifx.ArcadeDrive;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.GearShifter;

public class ToggleAutoShiftCmd extends CommandBase {
    private GearShifter shifter;
    private ArcadeDrive drive;
    private DriverControls dc;

    public ToggleAutoShiftCmd(DriverControls dc, GearShifter shifter, ArcadeDrive drive) {
        this.shifter = shifter;
        this.drive = drive;
        this.dc = dc;      //don't register, called in periodic

        addRequirements(drive, shifter);
    }

    @Override
    public void execute() {
        if (CommandScheduler.getInstance().getDefaultCommand(shifter) != null
                && CommandScheduler.getInstance().getDefaultCommand(shifter).getName().equals("AutomaticGearShift")) {
            shifter.setAutoShift(false);
            CommandScheduler scheduler = CommandScheduler.getInstance();
            scheduler.setDefaultCommand(shifter, null);
        } else {
            CommandScheduler.getInstance().setDefaultCommand(shifter,
             new AutomaticGearShiftCmd(dc, drive, shifter));
            shifter.setAutoShift(true);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}