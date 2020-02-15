package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.TankDrive;

public class TankDriveCmd extends CommandBase {
    private TankDrive drive;
    private DriverControls dc;

    @Override
    public boolean isFinished() {
        return false;
    }

    public TankDriveCmd(final DriverControls dc, final TankDrive drive) {
        this.drive = drive;
        this.dc = dc;
        addRequirements(drive);

        Robot.command = "Tank Drive";
    }

    public void execute() {
        drive.tankDrive(dc.getVelocityLeft(), dc.getVelocityRight());
    }

}