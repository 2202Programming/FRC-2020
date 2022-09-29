package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GearShifter;

public class ToggleAutoShiftCmd extends InstantCommand {
    private GearShifter shifter;

    /*
     * Instant command to toggle the autoshift on the shifter It doesn't care what
     * shifting algorithm (cmd) is being used, as this simply enables/disables on
     * the sub-system.
     * 
     * Any autoshift control commands will check that value before engaging any
     * automatic up/down requests.
     * 
     */
    public ToggleAutoShiftCmd(GearShifter shifter) {
        this.shifter = shifter;
    }

    @Override
    public void execute() {
        @SuppressWarnings("unused")
        boolean status = (shifter.isAutoShiftEnabled()) ? shifter.disableAutoShift() : shifter.enableAutoShift();
    }
}