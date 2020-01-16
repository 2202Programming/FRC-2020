package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;;
import frc.robot.Robot;
import frc.robot.subsystems.GearShifter.Gear;

public class ShiftGear extends InstantCommand {
    private Gear g;

    public ShiftGear(Gear g) {
        requires(Robot.gearShifter);
        this.g = g;
    }

    protected void execute() {
        if (g == Gear.HIGH_GEAR) {
            Robot.gearShifter.shiftUp();
        } else {
            Robot.gearShifter.shiftDown();
        }
    }
}