// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine_Subsystem;

/**
 * Simple command to set power cell count.  Done this way
 * so it can run when when disabled.
 */
public class SetPowerCellCount extends InstantCommand {
  final int m_count;
  final Magazine_Subsystem m_mag;

  public SetPowerCellCount(int pcCount) {
    m_mag = RobotContainer.getInstance().intake.getMagazine();
    m_count = pcCount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  m_mag.setPC(m_count);  }

  //Safe to run when disabled, true!
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
