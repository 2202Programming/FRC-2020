package frc.robot.commands.challenge;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePosition.Direction;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.ifx.VelocityDrive;


public class GalacticSearch extends SequentialCommandGroup {
  // trajectories we will run, selected by isA flag
  Trajectory startTraj;
  Trajectory blueTraj;
  Trajectory redTraj;

  VelocityDrive drive;
  Intake_Subsystem intake;
  Magazine_Subsystem magazine;

  // Creates a new GalacticSearch object
  public GalacticSearch(VelocityDrive drive, boolean isA) {
    RobotContainer rc = RobotContainer.getInstance();
    this.drive = drive;
    this.intake = rc.intake;
    this.magazine = rc.intake.getMagazine();  
    // magazine default command will control intake
    addRequirements(drive);
   
    // paths get read in can be looked up by simple name
    if (isA) {
      startTraj = rc.getTrajectory("SearchAStart");
      blueTraj = rc.getTrajectory("SearchABlue");
      redTraj = rc.getTrajectory("SearchARed.");
    } else {
      startTraj =  rc.getTrajectory("SearchBStart");
      blueTraj = rc.getTrajectory("SearchBBlue");
      redTraj =  rc.getTrajectory("SearchBRed");
    }
    /**
      *   Follow starting trajectory and then see if we found a PC.
      *   Use a conditional Command to then execute either Red or Blue paths
      *   based on what we found.
      */
      this.addCommands(
        new InstantCommand( ()->  { magazine.setPC(0); } ),
        new IntakePosition(intake, Direction.Down),
        new IntakePower(intake, Power.On, 0.5),
        new followTrajectory(drive, startTraj).andThen(new PrintCommand("GS-Start trajectory done.")), 
        new ConditionalCommand(
            // on true, magEmpty, found nothing do blue
            new PrintCommand("GS-Blue Trajectory").andThen(new followTrajectory(drive, blueTraj)),  
            // on False, found something do read
            new PrintCommand("GS-found PC,Red Trajectory").andThen(new followTrajectory(drive, redTraj)),   
            magazine::isMagEmpty)         // conditional
      );

  }
}