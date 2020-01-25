package frc.robot.commands.drive;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;
;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GearShifter;
import frc.robot.util.ExpoShaper;

public class ArcadeDriveVelCmd implements Command {
    private ExpoShaper speedShaper;
    private ExpoShaper rotationShaper;
    private Drivetrain drive;
    private GearShifter gearbox;

    private XboxController xbox;
    //### hack constants - move to better spot when working
    private double Krpm = 0.0;  

    @Override
    public boolean isFinished() {
        return false;
    }

    public ArcadeDriveVelCmd(Drivetrain drive, GearShifter gearbox, XboxController controller) {
        this.drive = drive;
        this.gearbox = gearbox;
        xbox = controller;
        speedShaper = new ExpoShaper(0.1);
        rotationShaper = new ExpoShaper(0.1);
    }
    
    @Override
    public void execute() {
        // Robot.driveTrain.ArcadeDrive(0.90, 0, true);
        double s = speedShaper.expo(xbox.getY(Hand.kLeft));
        // soften the input by limiting the max input
        double rot = rotationShaper.expo(0.8 * xbox.getX(Hand.kRight));
        drive.arcadeDrive(s, rot);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> subs = new HashSet<Subsystem>();
        subs.add(drive);
        subs.add(gearbox);
        return subs;
    }
    @Override
    public void initialize() {

    }

}