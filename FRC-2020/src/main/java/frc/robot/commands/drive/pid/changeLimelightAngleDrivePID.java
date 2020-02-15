package frc.robot.commands.drive.pid;

import java.util.Set;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drive.DriveWithLimelightToDistanceDegCmd;

public class changeLimelightAngleDrivePID extends InstantCommand implements Sendable {

    public changeLimelightAngleDrivePID(final char valueToChange, final double incrementValue) {
        this.valueToChange = valueToChange;
        this.incrementValue = incrementValue;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub

    }

    private final char valueToChange;
    private final double incrementValue;
}