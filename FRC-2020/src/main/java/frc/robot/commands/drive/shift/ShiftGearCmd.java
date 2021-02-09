package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.Shifter.Gear;

/**
 * ShiftGearCmd - requests a shift to the given gear.
 */
public class ShiftGearCmd extends InstantCommand {
    private Gear g;
    private Shifter shifter;

    public ShiftGearCmd(Shifter shifter, Gear g) {
        this.shifter = shifter;
        this.g = g;
    }
    @Override
    public void initialize() {
        if (g == Gear.HIGH) {
            shifter.shiftUp();
        } else {
            shifter.shiftDown();
        }
    }
}