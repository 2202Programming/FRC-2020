package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.ArcadeDrive;
import frc.robot.subsystems.ifx.DriverControls;

public class ArcadeDriveCmd extends CommandBase {
    private final ArcadeDrive drive;
    private final DriverControls dc;

    public ArcadeDriveCmd(final DriverControls dc, final ArcadeDrive drive) {
         this.drive = drive;
        this.dc = dc;

        addRequirements(drive);
    }

    public void execute() {
        final double vel = dc.getVelocity();
        final double rot = dc.getRotation();

        drive.arcadeDrive(vel, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}