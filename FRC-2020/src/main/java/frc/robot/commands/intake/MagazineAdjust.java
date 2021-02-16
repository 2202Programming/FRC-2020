/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import static frc.robot.Constants.DT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine_Subsystem;

public class MagazineAdjust extends CommandBase {
  private Magazine_Subsystem magazine;
  private double strength = 0.8;
  private int pulseCounts = -1;
  private int counts; // count down timer
  /**
   * Creates a new MagazineAdjust.
   */
  public MagazineAdjust(Magazine_Subsystem mag, boolean forward, double pulseTime) {
    magazine = mag;

    //set the direction
    strength *= (forward) ? 1.0 : -1.0;
    pulseCounts = (pulseTime > 0.0) ? (int) ((pulseTime / DT) + 1) : -1;
    addRequirements(mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    counts = pulseCounts;
    magazine.beltOn(strength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counts--;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    magazine.beltOff();
  }

  // Returns true when the command should end.
  // This command ends with button release.
  @Override
  public boolean isFinished() {
    // we countdown to zero if a pulse is given.
    return (counts == 0);
  }
}
