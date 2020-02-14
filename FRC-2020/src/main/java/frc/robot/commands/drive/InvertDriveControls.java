/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ifx.DriverControls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InvertDriveControls extends InstantCommand {
  public DriverControls dc;
  public InvertDriveControls(DriverControls dc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dc = dc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (dc.isControlInverted()) {
      dc.setInvertControls(false);
    } else {
      dc.setInvertControls(true);
    }
  }
}
