package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArcadeDrive;
import frc.robot.util.ExpoShaper;

public class ArcadeDriveCmd extends CommandBase {
    private ExpoShaper speedShaper;
    private ExpoShaper rotationShaper;
    private ArcadeDrive drive;
    private XboxController xbox;

    public ArcadeDriveCmd(ArcadeDrive drive, XboxController controller) {
        this.drive = drive;
        xbox = controller;
        speedShaper = new ExpoShaper(0.6);
        rotationShaper = new ExpoShaper(0.5);

        addRequirements(drive);
    }

    public void execute() {
        // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
        double s = 0.8 * speedShaper.expo(xbox.getY(Hand.kLeft));
        // soften the input by limiting the max input
        double rot = 0.8 * rotationShaper.expo(xbox.getX(Hand.kRight));
        drive.arcadeDrive(s, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}