package frc.robot.commands.drive.pid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.DriveWithLimelightToDistanceDegCmd;

public class changeLimelightAngleDrivePID extends InstantCommand {

    public changeLimelightAngleDrivePID(final char valueToChange, final double incrementValue) {
        this.valueToChange = valueToChange;
        this.incrementValue = incrementValue;
    }

    @Override
    public void initialize() {
        switch (Character.toUpperCase(valueToChange)) {
        case 'P':
            DriveWithLimelightToDistanceDegCmd.setKap(incrementValue);
            break;
        case 'I':
            DriveWithLimelightToDistanceDegCmd.setKai(incrementValue);
            break;
        case 'D':
            DriveWithLimelightToDistanceDegCmd.setKad(incrementValue);
            break;
        }
    }

    private final char valueToChange;
    private final double incrementValue;
}