package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;

public class ShiftGear extends InstantCommand {
    private Gear g;
    private GearShifter shifter;

    public ShiftGear(GearShifter shifter, Gear g) {
        this.shifter = shifter;
        this.g = g;
    }

    public void execute() {
        if (g == Gear.HIGH_GEAR) {
            shifter.shiftUp();
        } else {
            shifter.shiftDown();
        }
    }
}