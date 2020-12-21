package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;

/**
 * ShiftGearCmd - requests a shift to the given gear.
 */
public class ShiftGearCmd extends InstantCommand {
    private Gear g;
    private GearShifter shifter;

    public ShiftGearCmd(GearShifter shifter, Gear g) {
        this.shifter = shifter;
        this.g = g;
        addRequirements(shifter);
    }
    @Override
    public void initialize() {
        if (g == Gear.HIGH_GEAR) {
            shifter.shiftUp();
        } else {
            shifter.shiftDown();
        }
    }
}