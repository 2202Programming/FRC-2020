package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithLidarToDistanceDegCmd;
import frc.robot.commands.ShooterOnWithDelay;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class auto_cmd_group extends SequentialCommandGroup{
    
    double[] startDelay = {0.0, 3.0, 7.0, 15.0};  //made up number of seconds
    

    public auto_cmd_group(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
                            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget = 0;
        double maxSpeed = 10;
        double maxAngleSpeed = 10;
        double targetDistance = 0;
        double stopDist = 0;
        double targetVelocity = 0;
        boolean switch1 = false;
        boolean switch2 = false;
        double delay = 0;

        //TODO: Compute delay based on switches 
        int delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);        // sw 1 & 2
        int positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C)>>2;  // sw 3 & 4

        //todo: use positionCode to create the path

        addCommands(
            new WaitCommand(startDelay[delayCode]),
            new auto_creep_cmd(drive, limelight, angleTarget, maxSpeed, maxAngleSpeed, targetDistance),
            new auto_delay_cmd(switch1, switch2),
            new auto_limelightDrive_cmd(drive, limelight, lidar, stopDist, angleTarget, maxSpeed, targetVelocity),
            new auto_limelightLidar_cmd(drive, limelight, stopDist, angleTarget, maxSpeed, targetVelocity),
            new DriveWithLidarToDistanceDegCmd(drive, lidar, stopDist, angleTarget, maxSpeed),
            new ShooterOnWithDelay(intake, delay),
            new DriveWithLidarToDistanceDegCmd(drive, lidar, stopDist, angleTarget, maxSpeed),
            new auto_limelightLidar_cmd(drive, limelight, stopDist, angleTarget, maxSpeed, targetVelocity)
        );
    }
}