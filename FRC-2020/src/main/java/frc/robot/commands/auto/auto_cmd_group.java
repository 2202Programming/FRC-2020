package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithLidarToDistanceDegCmd;
import frc.robot.commands.ShooterOn;
import frc.robot.commands.ShooterOnWithDelay;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class auto_cmd_group extends SequentialCommandGroup{
    
    public auto_cmd_group(VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
                            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget = 0;
        double maxSpeed = 0;
        double targetDistance = 0;
        double stopDist = 0;
        double targetVelocity = 0;
        boolean switch1 = false;
        boolean switch2 = false;
        double delay = 0;

        addCommands(
            new auto_creep_cmd(drive, limelight, angleTarget, maxSpeed, targetDistance),
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