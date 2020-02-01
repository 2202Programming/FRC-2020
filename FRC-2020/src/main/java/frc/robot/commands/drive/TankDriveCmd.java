package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ExpoShaper;
import frc.robot.subsystems.TankDrive;

/**
 * An example command. You can replace me with your own command.
 */
public class TankDriveCmd extends CommandBase {
    private ExpoShaper speedShaperLeft;
    private ExpoShaper speedShaperRight;
    private TankDrive drive;
    private XboxController xbox;

    @Override
    public boolean isFinished() {
        return false;
    }
    public TankDriveCmd(TankDrive drive, XboxController controller) {
        this.drive = drive;
        xbox = controller;
        speedShaperLeft = new ExpoShaper(0.5);
        speedShaperRight = new ExpoShaper(0.5);

        addRequirements(drive);
    }
  
  public void execute() {
    // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
    double s1 = 0.8* speedShaperLeft.expo(xbox.getY(Hand.kLeft));
    // soften the input by limiting the max input
    double s2 = 0.8 * speedShaperRight.expo(xbox.getY(Hand.kRight));
    drive.tankDrive(s1, s2);
    }

}