package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;

public class ThrottledUpShift extends ParallelCommandGroup {
    public ThrottledUpShift(DriveTrain drive, GearShifter shifter) {
        addCommands(
            new ShiftGear(shifter, Gear.HIGH_GEAR),
            new ThrottleDriveInput(drive, 1.0, 0.5, 1));
    }
}