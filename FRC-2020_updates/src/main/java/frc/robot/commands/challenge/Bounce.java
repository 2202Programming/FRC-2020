/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.challenge;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.subsystems.ifx.VelocityDrive;

public class Bounce extends  SequentialCommandGroup  {
  RobotContainer rc = RobotContainer.getInstance();
   Trajectory[] trajs = { 
      rc.getTrajectory("Bounce1"),
      rc.getTrajectory("Bounce2"),
      rc.getTrajectory("Bounce3"),
      rc.getTrajectory("BounceEnd"), };

      VelocityDrive drive;
  /**
   * Creates a new Bounce.
   */
  public Bounce() {

    drive = rc.driveTrain;
    for (Trajectory t : trajs)
      this.addCommands( new followTrajectory(drive, t)  );
  }
}
