/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
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
	
	// starting position
	Trajectory[] shootHighStart = {Constants.HIGH_A, Constants.HIGH_B, Constants.HIGH_C};

	// backup pickup
	Trajectory[] pickupAfterShoot = {Constants.TRENCH_A, Constants.TRENCH_B, Constants.SHIELD_B, Constants.SHIELD_C};
	/**
	 * Creates a new TrajectoryBasedAuto.
	 */
	public TrajectoryBasedAuto(DriverControls dc, VelocityDifferentialDrive_Subsystem drive, Intake_Subsystem intake) {
		int delayCode;
		double delay;
		int positionCode;
		boolean highMode;
		boolean trenchMode;

		// Compute delay based on switches 1 and 2 (1 is first bit, 2 is second)
		delayCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x03);
		delay = startDelay[delayCode];
		SmartDashboard.putNumber("Auto: Delay", delayCode*3);

		// Get position based on switches 3 and 4 (3 is first bit, 4 is second)
		positionCode = (dc.getInitialButtons(Id.SwitchBoard) & 0x0C) >> 2; // sw 3 & 4
		SmartDashboard.putNumber("Auto: Position Code", positionCode);

		// Switch 5 Trench Mode
		trenchMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x10) >> 4 == 1) ? true : false;
		SmartDashboard.putBoolean("Auto: Trench Mode", trenchMode);

		// Switch 6 High Goal
		highMode = ((dc.getInitialButtons(Id.SwitchBoard) & 0x20) >> 5 == 1) ? true : false;
		SmartDashboard.putBoolean("Auto: High Goal", highMode);

		if (delayCode > 0) {
			addCommands(
				new WaitCommand(delay)
			);
		}

	}
}
