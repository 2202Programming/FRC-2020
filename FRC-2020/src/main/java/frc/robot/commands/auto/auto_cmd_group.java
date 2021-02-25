package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePosition.Direction;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.Shoot;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

public class auto_cmd_group extends SequentialCommandGroup {

    // startup delay
    double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };

    // limelight approach angle target
    double[] startAngle = { 0.0, Constants.ANGLE_A, Constants.ANGLE_B, Constants.ANGLE_C };

    // limelight approach distance target
    double[] limelightArea = { 0.0, Constants.AREA_A, Constants.AREA_B, Constants.AREA_C };

    // limelight departure angle target
    double[] limelightDepartureAngle = { 0.0, Constants.LIMELIGHT_DEPARTURE_ANGLE_A,
            Constants.LIMELIGHT_DEPARTURE_ANGLE_B, Constants.LIMELIGHT_DEPARTURE_ANGLE_C };

    // lidar departure angle target
    double[] lidarDepartureAngle = { 0.0, Constants.LIDAR_DEPARTURE_ANGLE_A, Constants.LIDAR_DEPARTURE_ANGLE_B,
            Constants.LIDAR_DEPARTURE_ANGLE_C };

    // limelight departure distance target
    double[] departureArea = { 0.0, Constants.DEPARTURE_AREA_A, Constants.DEPARTURE_AREA_B,
            Constants.DEPARTURE_AREA_C };

    public auto_cmd_group(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget;
        double delay;
        int delayCode;
        int positionCode;
        double limelightAreaTarget;
        boolean highMode;
        boolean trenchMode;

        // Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
        delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);
        
        // Get position based on switches 3 and 4 (3 is first bit, 4 is second)
        positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C) >> 2; // sw 3 & 4
        SmartDashboard.putNumber("Auto: Position Code", positionCode);

        // Switch 5 Trench Mode
        trenchMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x10) >> 4 == 1) ? true : false;
        SmartDashboard.putBoolean("Auto: Trench Mode", trenchMode);

        //Switch 6 High Goal
        highMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x20) >> 5 == 1) ? true : false;
        SmartDashboard.putBoolean("Auto: High Goal", highMode);


        // Test Code
        /**
         * delayCode = 0; // this should be removed to read in from switch positionCode
         * = 3; // thi0s should be removed to read in from switch //1 = A (far right)
         * closest to wall), 2 = B (centeted), 3 = C (far left) trenchMode = false; //
         * this should be removed to read in from switch
         */

        delay = startDelay[delayCode];
        SmartDashboard.putNumber("Auto: Delay (secs)", delay+3); //add 3 b/c the driveoffline takes 3 secs

        angleTarget = startAngle[positionCode];
        limelightAreaTarget = limelightArea[positionCode];

        if (delayCode > 0) { // do these only if not first in attack order
            addCommands(new DriveOffLine(drive, -0.9).withTimeout(1.5), new DriveOffLine(drive, 0.9).withTimeout(1.5),
                    new WaitCommand(delay));
        }

        //Just drive off line backwards if not in position A,B, or C
        //(or if we don't want to interfere at all)
        if (positionCode == 0) {
            addCommands(new DriveOffLine(drive, -0.9).withTimeout(3.7));
        }
        //TODO: Make this more legit
        else if (highMode) {
            addCommands(
                    //TODO: Replace with lidar or limelight to get exact shooting distance
                    new DriveOffLine(drive, 0.8).withTimeout(2.7),
                    new IntakePosition(intake, Direction.Down),
                    new WaitCommand(0.7),
                    new MagazineAngle(intake, 40.0).withTimeout(2.0),
                    new WaitCommand(1.2),
                    new Shoot(intake, ShooterOnCmd.dataLow).withTimeout(3), //need to add Data type to parser
                    new MagazineAngle(intake, 25.0).withTimeout(2.0),
                    new IntakePosition(intake, Direction.Up)
            );
        }
        else {
            addCommands(

                // Move forward using limelight to a certain limelight area(distance estimate)
                new auto_creep_area_cmd(drive, limelight, lidar, angleTarget, 3, 60, limelightAreaTarget, true)
                        .withTimeout(3), //double variables with Java reflections?

                // Drive open loop forward until lidar valid
                new auto_drive_straight_until_lidar_cmd(drive, lidar, 2).withTimeout(3),

                // Drive forward at current angle using lidar
                new auto_drive_lidar(drive, lidar, 300, 1.25, true, lidarDepartureAngle[positionCode]),

                // Drive forward at to zero angle using lidar
                // Timeout to shoot otherwise we don't make it in time
                new auto_drive_lidar(drive, lidar, 120, 3, true, 0).withTimeout(2),

                new ParallelCommandGroup( // wall pressure + deployment simultaneously
                        new auto_drive_straight_cmd(drive, 1.5).withTimeout(2), // pressure on wall during dump

                        // Deploy balls
                        new Shoot(intake).withTimeout(2) // turn shooter on for 2 seconds
                ),

                // Drive backwards at departure angle using lidar
                // 1.5 multiplier hack to get robot further out of the way
                new auto_drive_lidar(drive, lidar, 800, 3, false, 1.5*lidarDepartureAngle[positionCode]),

                // Drive open loop backwards until limelight valid
                new auto_drive_straight_cmd(drive, -1).withTimeout(1),

                // Move backwards using limelight to a certain limelight area(distance estimate)
                new auto_creep_area_cmd(drive, limelight, lidar, limelightDepartureAngle[positionCode], 3, 60,
                        departureArea[positionCode], false),

                // Drive open loop backwards until limelight valid
                new auto_drive_straight_cmd(drive, -2).withTimeout(0.8), new toggleLED(limelight)

            );

            if (trenchMode) { // only move to pickup balls after first 3 are delivered if switch enables this
                              // behavior
                addCommands( //should this be within non-HighMode only?

                        // turn only with limelight to face balls to pick up
                        new auto_creep_area_turn_cmd(drive, limelight, -24, 60),

                        // drive forwards and pick up balls
                        new auto_ball_capture_cmd(intake, drive, 0.5, 0.7, -4).withTimeout(4));
            }
        }

    }
}