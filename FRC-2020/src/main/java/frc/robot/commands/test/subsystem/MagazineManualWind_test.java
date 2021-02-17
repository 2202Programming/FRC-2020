// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test.subsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineManualWind_test extends CommandBase {
    MagazinePositioner  magPositioner; 
    double rpm;

  /** Creates a new MagazineManualWind_test. */
  public MagazineManualWind_test(Intake_Subsystem intake, double rpm) {
    magPositioner = intake.getMagazine().getMagPositioner();
    this.rpm = rpm;
    addRequirements(magPositioner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    magPositioner.wind(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (magPositioner.isAtBottom() || magPositioner.isAtTop()) {
      magPositioner.stop();
      return true;
    }
    return false;
  }
}
