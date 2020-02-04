package frc.robot.commands.drive;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class SwitchDriveMode implements Command
{
    public final DriveTrain driveTrain;
    public final XboxController driver;
    private final ArcadeDrive arcade;
    private final TankDrive tank;
    private final Command currentMode;
    public SwitchDriveMode (DriveTrain driveTrain, ArcadeDrive arcade, TankDrive tank, XboxController driver)
    {
        currentMode = CommandScheduler.getInstance().getDefaultCommand(RobotContainer.driveTrain);
        this.driveTrain = driveTrain;
        this.driver = driver;
        this.arcade = arcade;
        this.tank = tank;
       
    }
    public void execute()
    {
        if (currentMode.equals(RobotContainer.arcade))
        {
            CommandScheduler.getInstance().setDefaultCommand(driveTrain, tank);
        }
        else {
            CommandScheduler.getInstance().setDefaultCommand(driveTrain, arcade);
        }
    }

    public boolean isFinished()
    {
        return true;
    }

    @Override
	public Set<Subsystem> getRequirements() {
		Set<Subsystem> subs = new HashSet<Subsystem>();
    subs.add(driveTrain);
    return subs;
	}

}