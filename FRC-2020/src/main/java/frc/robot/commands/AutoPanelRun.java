/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.panel.SetPanelArmExtension;
import frc.robot.commands.panel.SimpPositionControl;
import frc.robot.commands.panel.SimpRotateControl;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Control_Panel;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoPanelRun extends SequentialCommandGroup {
  /**
   * Creates a new AutoPanelRun.
   */
  public AutoPanelRun(VelocityDifferentialDrive_Subsystem driveTrain, Control_Panel panel, Color_Subsystem detector) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    Trajectory pathToPanel = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(0, 1),
        new Translation2d(1, 2)
      )
      ,new Pose2d(1, 3, new Rotation2d(0)), driveTrain.TRAJ_CONFIG);


    addCommands(

      //Put the panel arm up
      new SetPanelArmExtension(panel, true),

      //Drive on a given trajectory until the limit switch triggers
      new DriveOnTrajectory(driveTrain, pathToPanel).withInterrupt(panel::getLimitSwitch)
    );

    //Determine rotation or position control based on field data or on button
    boolean rotationControl = true;/*new XboxController(3).getRawButton(12);*/

    //Do either rotation or position control
    if (rotationControl) {
      addCommands(new SimpRotateControl(panel));
    } else {
      addCommands(new SimpPositionControl(panel, detector));
    }

    addCommands(
      //Retract panel arm
      new SetPanelArmExtension(panel, false)
    );
  }
}
