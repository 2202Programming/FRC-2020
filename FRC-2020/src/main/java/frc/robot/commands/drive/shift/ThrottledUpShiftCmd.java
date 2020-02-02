package frc.robot.commands.drive.shift;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ifx.ArcadeDrive;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;

public class ThrottledUpShiftCmd extends ParallelCommandGroup {
    public ThrottledUpShiftCmd(ArcadeDrive drive, GearShifter shifter) {
        addCommands(
            new ShiftGearCmd(shifter, Gear.HIGH_GEAR),
            new ThrottleDriveInputCmd(drive, 1.0, 0.5, 1));
    }
}