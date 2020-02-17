package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithLidarToDistanceDegCmd;
//import frc.robot.commands.ShooterOnWithDelay;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.Constants;

public class auto_cmd_group extends SequentialCommandGroup{
    
    double[] startDelay = {0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C};
    
    public auto_cmd_group(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
                            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget = 0;
        double maxSpeed = 10;
        double maxAngleSpeed = 10;
        double targetDistance = 0;
        double stopDist = 0;
        double targetVelocity = 0;
        double delay = 0;

        // Compute delay based on switches 
        int delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);        // sw 1 & 2

        // TODO: Change path based on positionCode from switch
        // int positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C)>>2;  // sw 3 & 4

        delay =startDelay[delayCode];

        addCommands(
            new WaitCommand(delay),
            new auto_creep_cmd(drive, limelight, angleTarget, maxSpeed, maxAngleSpeed, targetDistance),
            //     new auto_delay_cmd(switch1, switch2),
            new auto_limelightDrive_cmd(drive, limelight, lidar, stopDist, angleTarget, maxSpeed, targetVelocity),
            new auto_limelightLidar_cmd(drive, limelight, stopDist, angleTarget, maxSpeed, targetVelocity),
            new DriveWithLidarToDistanceDegCmd(drive, lidar, stopDist, angleTarget, maxSpeed),
            new ShooterOn(intake, 1200, 0.4).withTimeout(4.0),  //turn shooter on for 4 seconds 1200 rpm
            //new ShooterOnWithDelay(intake, delay),
            new DriveWithLidarToDistanceDegCmd(drive, lidar, stopDist, angleTarget, maxSpeed),
            new auto_limelightLidar_cmd(drive, limelight, stopDist, angleTarget, maxSpeed, targetVelocity)
        );
    }
}