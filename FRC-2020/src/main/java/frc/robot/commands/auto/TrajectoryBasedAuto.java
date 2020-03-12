/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveOnTrajectory;
import frc.robot.commands.intake.IntakeToggleCmd;
import frc.robot.commands.intake.MagazineToggleCmd;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.commands.intake.ToggleIntakeRaised;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrajectoryBasedAuto extends SequentialCommandGroup {
	// startup delay
	double[] startDelay = {0.0, Constants.DELAY_A, Constants.DELAY_B, Constants.DELAY_C};

	/**
     *  TRAJECTORIES FOR AUTO
     */

    //Drive straight at angle 
    public Trajectory HIGH_A = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(30)),//Start
        List.of(
        ),
        new Pose2d(),                               // End
        Constants.TRAJ_CONFIG);
    
    
        //Drive straight
    public Trajectory HIGH_B = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),//Start
        List.of(
        ),
        new Pose2d(0.5, 0, Rotation2d.fromDegrees(0)),                               // End
        Constants.TRAJ_CONFIG);

    //Drive straight at angle
    public Trajectory HIGH_C = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(-30)),//Start
        List.of(
        ),
        new Pose2d(),                               // End
        Constants.TRAJ_CONFIG);

    //Trajectories for auto backup to grab more balls
    public Trajectory TRENCH_A = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),//Start
        List.of(                                    // Waypoints
            new Translation2d()
        ),
        new Pose2d(),                               // End
        Constants.TRAJ_CONFIG);
    
    public Trajectory TRENCH_B = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),//Start
        List.of(                                    // Waypoints
            new Translation2d()
        ),
        new Pose2d(),                               // End
		Constants.TRAJ_CONFIG);

    public Trajectory SHIELD_C = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),//Start
        List.of(                                    // Waypoints
            new Translation2d()
        ),
        new Pose2d(),                               // End
        Constants.TRAJ_CONFIG);
	
	// starting position
	Trajectory[] pathToShoot = {null, HIGH_A, HIGH_B, HIGH_C};

	// backup pickup
	Trajectory[] pickupAfterShoot = {TRENCH_A, TRENCH_B, SHIELD_C};

	// drive back to shooting position
	Trajectory[] shootAfterPickup = {};
	/**
	 * Creates a new TrajectoryBasedAuto.
	 */
	public TrajectoryBasedAuto(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake) {
		int delayCode;
		int positionCode;
		boolean pickupMode;
		boolean highMode;

		// Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
		delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);
		double delay = startDelay[delayCode];
		SmartDashboard.putNumber("Auto: Delay", delayCode*3);

		// Get position based on switches 3 and 4 (3 is first bit, 4 is second)
		positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C) >> 2; // sw 3 & 4
		SmartDashboard.putNumber("Auto: Position Code", positionCode);

		// Switch 5 Pick up more balls after shooting Mode
		pickupMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x10) >> 4 == 1) ? true : false;
		SmartDashboard.putBoolean("Auto: Trench Mode", pickupMode);

		// Switch 6 High Goal
		highMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x20) >> 5 == 1) ? true : false;
		SmartDashboard.putBoolean("Auto: High Goal", highMode);

		//TESTING USE ONLY. WHEN SWITCHBOARD UNAVAILABLE
		delayCode = 0;
		positionCode = 2;
		pickupMode = false;
		highMode = true;

		if (delayCode > 0) {
			addCommands(
				new WaitCommand(delay)
			);
		}

		//Position 0: Just Drive Off Line
		if (positionCode == 0) {
			addCommands(new DriveOffLine(drive, -0.7));
		}

		//Not position 0, actually do stuff
		else {
			addCommands(
				//Drive in position to shoot
				new DriveOnTrajectory(drive, pathToShoot[positionCode]),
				new WaitCommand(0.5),

				//Drop intake, raise magazine
				new ToggleIntakeRaised(intake),
				new WaitCommand(0.5),
				new MagazineToggleCmd(intake),
				new WaitCommand(0.3),

				//Shoot
				new ShooterOn(intake, 0.6, 1, 0.5).withTimeout(1)
			);
		}

		//Go pickup more ballssss
		if(pickupMode) {
			addCommands(

				//Drive path back to balls while running intake and magazine
				new ParallelCommandGroup(
					new DriveOnTrajectory(drive, pickupAfterShoot[positionCode]),
					new IntakeToggleCmd(intake, 0.6, 0.6) //TODO: Find better way to make sure balls get in
				),

				//Drive back into position to shoot
				new DriveOnTrajectory(drive, shootAfterPickup[positionCode]),
				new WaitCommand(0.5),

				//Drop intake, raise magazine
				new ToggleIntakeRaised(intake),
				new WaitCommand(0.5),
				new MagazineToggleCmd(intake),
				new WaitCommand(0.3),

				//Shoot
				new ShooterOn(intake, 0.6, 1, 0.5).withTimeout(1)
			);
		}

	}
}
