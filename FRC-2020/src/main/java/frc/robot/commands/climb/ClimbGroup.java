/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ClimbGroup extends SequentialCommandGroup {

  private static final double ROT_SPEED = 0.5;
  private static final double WINCH_SPEED = 1;
  /**
   * Creates a new ClimbGroup.
   */
  public ClimbGroup(ClimberSubsystem climber) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new SetClimbArmRotation(climber, ROT_SPEED).withTimeout(1),
      new SetClimbArmExtension(climber, true),
      new WaitCommand(0.5),
      new SetClimbArmExtension(climber, false),
      new RunWinch(climber, WINCH_SPEED)
    );
  }
}
