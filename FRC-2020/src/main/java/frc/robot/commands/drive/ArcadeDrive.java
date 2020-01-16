package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ExpoShaper;

public class ArcadeDrive implements Command {
    private ExpoShaper speedShaper;
    private ExpoShaper rotationShaper;
    private Drivetrain drive;

    @Override
    public boolean isFinished() {
        return false;
    }

    public ArcadeDrive(Drivetrain drive) {
        this.drive = drive;
    }

    public void execute() {
        // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
        double s = speedShaper.expo(RobotContainer.driver.getY(Hand.kLeft));
        // soften the input by limiting the max input
        double rot = rotationShaper.expo(0.8 * RobotContainer.driver.getX(Hand.kRight));
        drive.arcadeDrive(s, rot);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subs = new HashSet<Subsystem>();
        subs.add(drive);
        return subs;
    }

}