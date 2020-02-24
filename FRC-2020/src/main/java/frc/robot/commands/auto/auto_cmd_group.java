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
import frc.robot.Robot;

public class auto_cmd_group extends SequentialCommandGroup {

    double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };

    public auto_cmd_group(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget = 0;
        double maxSpeed = 10;
        double maxAngleSpeed = 10;
        double targetDistance = 0.1;
        double stopDist = 0;
        double targetForwardPower = 0.1;
        double delay = 0;
        double departure_angle = 20;

        // Compute delay based on switches
        int delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03); // sw 1 & 2
        // int positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C)>>2; // sw 3
        // & 4

        // TODO: Change path based on positionCode from switch
        // int positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C)>>2; // sw 3
        // & 4

        delay = startDelay[delayCode];

        addCommands(
                //Move forward using limelight to a certain limelight area(distance estimate)
                new auto_creep_area_cmd(drive, limelight, lidar, -9, 0.4, 0.2, 2.5, true), 

                //Drive open loop forward until lidar valid
                new auto_drive_straight_until_lidar_cmd(drive, lidar, 0.2),
                
                //Set global variable for current angle with lidar, to help with later retreat angle
                new set_departure_angle(lidar.findAngle()),

                //Drive forward at fixed angle using lidar
                new auto_drive_lidar(drive, lidar, 600, Robot.departureAngle, 0.1, true),

                //Drive forward rest of the way at angle zero (flatten to wall)
                new auto_drive_lidar(drive, lidar, 400, 0, 0.07, true),

                //Deploy balls
                new ShooterOn(intake, 1200, 0.4).withTimeout(2.0), // turn shooter on for 2 seconds 1200 rpm

                //Retreat at angle zero for fixed amount of time (limelight doesn't work too close to wall due to washout)
                new auto_drive_lidar(drive, lidar, 700, 0, 0.2, false).withTimeout(2),

                //Drive backwards until limelight valid
                new auto_drive_straight_until_limelight_cmd(drive, lidar, limelight, -0.2),

                //Drive backwards open loop a bit to make sure limelight is fully in view
                new auto_drive_straight_cmd(drive, lidar, -0.2).withTimeout(2),

                //Drive backwards at angle (departure angle?) until limelight area small enough (far enough)
                new auto_creep_area_cmd(drive, limelight, lidar, -15, 0.3, 0.2, 1.6, false)

        );

        /*
         * addCommands( new WaitCommand(delay), new auto_creep_area_cmd(drive,
         * limelight, lidar, angleTarget, maxSpeed, maxAngleSpeed, targetDistance), //
         * new auto_delay_cmd(switch1, switch2), new auto_limelightDrive_cmd(drive,
         * limelight, lidar, stopDist, angleTarget, maxSpeed, targetForwardPower), //
         * drive towards target with limelight until lidar valid new
         * set_departure_angle(lidar.findAngle()), //sets global variable in Robot new
         * auto_drive_lidar(drive, lidar, 12, Robot.departureAngle, 0.1), // drive
         * straight with lidar alone at current angle until XX inches from wall new
         * auto_drive_lidar(drive, lidar, 5, 0, 0.1), //drive to angle 0, stop at XX
         * inches new ShooterOn(intake, 1200, 0.4).withTimeout(4.0), //turn shooter on
         * for 4 seconds 1200 rpm //new ShooterOnWithDelay(intake, delay), new
         * auto_drive_lidar_until_limelight(drive, lidar, limelight,
         * Robot.departureAngle, 0.1), //drive at angle departure_angle, go until
         * limelight valid new auto_limelightLidar_cmd(drive, limelight, stopDist,
         * angleTarget, maxSpeed, targetForwardPower) );
         */
    }
}