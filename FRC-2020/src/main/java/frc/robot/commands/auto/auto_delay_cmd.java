/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class auto_delay_cmd extends CommandBase {
  /**
   * Creates a new auto_delay_cmd.
   */

   private double timeStarted;
   private double delay; //in milliseconds

  public auto_delay_cmd(boolean switch1, boolean switch2) {
    // Use addRequirements() here to declare subsystem dependencies.
    //assuming using two switches on driver's station, switch1 on means A, both off means B, and switch2 on means C
    if (switch1)
        delay = Constants.DELAY_A;
    else if (switch2)
        delay = Constants.DELAY_C;
    else
        delay = Constants.DELAY_B;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = System.currentTimeMillis();
    Robot.command = "Auto delay";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (System.currentTimeMillis() - timeStarted) >= delay;
  }
}
