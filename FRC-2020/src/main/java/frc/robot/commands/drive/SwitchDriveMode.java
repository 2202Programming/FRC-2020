package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class SwitchDriveMode
{
    public static final DriveTrain driveTrain = new DriveTrain();
    public static final XboxController driver = new XboxController(0);
    private final ArcadeDrive arcade = new ArcadeDrive(driveTrain, driver);
    private final TankDrive tank = new TankDrive(driveTrain, driver);
    private final Command currentMode;
    public SwitchDriveMode ()
    {
        currentMode = CommandScheduler.getInstance().getDefaultCommand(RobotContainer.driveTrain);
       
    }
    public void Switch()
    {
        if (currentMode.equals(RobotContainer.arcade))
        {
            CommandScheduler.getInstance().setDefaultCommand(driveTrain, tank);
        }
        else {
            CommandScheduler.getInstance().setDefaultCommand(driveTrain, arcade);
        }
    }

}