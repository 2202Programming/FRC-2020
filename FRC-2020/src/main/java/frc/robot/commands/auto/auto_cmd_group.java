package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.Constants;

public class auto_cmd_group extends SequentialCommandGroup {
    
    //startup delay
    double[] startDelay = { 0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C };

    //limelight approach angle target
    double[] startAngle = { 0.0, Constants.ANGLE_A, Constants.ANGLE_B, Constants.ANGLE_C };

    //limelight approach distance target
    double[] limelightArea = { 0.0, Constants.AREA_A, Constants.AREA_B, Constants.AREA_C };

    //limelight departure angle target
    double[] limelightDepartureAngle = { 0.0, Constants.LIMELIGHT_DEPARTURE_ANGLE_A, Constants.LIMELIGHT_DEPARTURE_ANGLE_B,
        Constants.LIMELIGHT_DEPARTURE_ANGLE_C };

    //lidar departure angle target
    double[] lidarDepartureAngle = { 0.0, Constants.LIDAR_DEPARTURE_ANGLE_A, Constants.LIDAR_DEPARTURE_ANGLE_B,
        Constants.LIDAR_DEPARTURE_ANGLE_C };

    //limelight departure distance target
    double[] departureArea = { 0.0, Constants.DEPARTURE_AREA_A, Constants.DEPARTURE_AREA_B, Constants.DEPARTURE_AREA_C };

    public auto_cmd_group(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake,
            Limelight_Subsystem limelight, Lidar_Subsystem lidar) {
        double angleTarget;
        double delay;
        int positionCode;
        double limelightAreaTarget;
        boolean trenchMode;

        // Compute delay based on switches
        int delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03); // sw 1 & 2
        
        // TODO: Change path based on positionCode from switch
        positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C)>>2; // sw 3 & 4

        //Switch 5
        trenchMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x10) == 1) ? true : false;

        delayCode = 0; // this should be removed to read in from switch
        positionCode = 2; // this should be removed to read in from switch
        //1 = A (far right closest to wall), 2 = B (centeted), 3 = C (far left)
        trenchMode = false; // this should be removed to read in from switch


        delay = startDelay[delayCode];
        angleTarget = startAngle[positionCode];
        limelightAreaTarget = limelightArea[positionCode];
        
        if (delay > 0) { // do these only if not first in attack order
            addCommands(
                new DriveOffLine(drive),
                new auto_do_nothing().withTimeout(delay));
        }

        addCommands(

                //Move forward using limelight to a certain limelight area(distance estimate) 
                new auto_creep_area_cmd(drive, limelight, lidar, angleTarget, 3, 60, limelightAreaTarget, true).withTimeout(3),

                //Drive open loop forward until lidar valid
                new auto_drive_straight_until_lidar_cmd(drive, lidar, 2).withTimeout(3),

                //Drive forward at current angle using lidar
                new auto_drive_lidar(drive, lidar, 200, 1.25, true, lidarDepartureAngle[positionCode]),

                //Drive forward at to zero angle using lidar
                new auto_drive_lidar(drive, lidar, 125, 1, true, 0),
                
                new ParallelCommandGroup( //wall pressure + deployment simultaneously
                    new auto_drive_straight_cmd(drive, 0.5).withTimeout(2), // pressure on wall during dump

                    //Deploy balls
                    new ShooterOn(intake, 1200, 0.4).withTimeout(2.0) // turn shooter on for 2 seconds 1200 rpm
                ),

                //Drive backwards at departure angle using lidar
                new auto_drive_lidar(drive, lidar, 800, 3, false, lidarDepartureAngle[positionCode]),

                //Drive open loop backwards until limelight valid
                new auto_drive_straight_cmd(drive, -3).withTimeout(1),

                //Move backwards using limelight to a certain limelight area(distance estimate) 
                new auto_creep_area_cmd(drive, limelight, lidar, angleTarget, 3, 60, departureArea[positionCode], false)


        );

        if(trenchMode){ //only move to pickup balls after first 3 are delivered if switch enables this behavior
            addCommands(
                
                    //turn only with limelight to face balls to pick up
                    new auto_creep_area_turn_cmd(drive, limelight, -29, 60),

                    //drive forwards and pick up balls
                    new auto_ball_capture_cmd(intake, drive, limelight, 0.75, 0.75, 60).withTimeout(10)
            );
        }

    }
}