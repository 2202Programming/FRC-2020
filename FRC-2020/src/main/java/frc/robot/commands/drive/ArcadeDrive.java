package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
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
    private XboxController xbox;

    @Override
    public boolean isFinished() {
        return false;
    }

    public ArcadeDrive(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        xbox = controller;
        speedShaper = new ExpoShaper(0.6);
        rotationShaper = new ExpoShaper(0.5);
    }

    public void execute() {
        // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
        double s = 0.8* speedShaper.expo(xbox.getY(Hand.kLeft));
        // soften the input by limiting the max input
        double rot = 0.8 * rotationShaper.expo(xbox.getX(Hand.kRight));
        drive.arcadeDrive(s, rot, false);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subs = new HashSet<Subsystem>();
        subs.add(drive);
        return subs;
    }

}