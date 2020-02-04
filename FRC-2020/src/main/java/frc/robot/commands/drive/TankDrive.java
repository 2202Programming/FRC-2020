package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.ExpoShaper;

/**
 * An example command. You can replace me with your own command.
 */
public class TankDrive implements Command {
    private ExpoShaper speedShaperLeft;
    private ExpoShaper speedShaperRight;
    private DriveTrain drive;
    private XboxController xbox;
    @Override
    public boolean isFinished() {
        return false;
    }
    public TankDrive(DriveTrain drive, XboxController controller) {
        this.drive = drive;
        xbox = controller;
        speedShaperLeft = new ExpoShaper(0.5);
        speedShaperRight = new ExpoShaper(0.5);
    }
  
  

  public void execute() {
    // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
    double s1 = 0.8* speedShaperLeft.expo(xbox.getY(Hand.kLeft));
    // soften the input by limiting the max input
    double s2 = 0.8 * speedShaperRight.expo(xbox.getY(Hand.kRight));
    drive.tankDrive(s1, s2, false);
}


@Override
public Set<Subsystem> getRequirements() {
    Set<Subsystem> subs = new HashSet<Subsystem>();
    subs.add(drive);
    return subs;
}
}