/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Control_Panel;

public class SimpRotateControl extends CommandBase {

  private static final double COUNTS_PER_WHEEL_ROTATION = 1024; //TODO: Find correct value

  //Intentionally overrotate to avoid undershoot, just need to be between 3 and 5 full rotations
  private static final double TARGET_PANEL_ROTATIONS = 3.5; 
  private static final double ROTATOR_WHEEL_CIRCUMFERENCE = 4 * Math.PI; //inches
  private static final double PANEL_CIRCUMFERENCE = 32 * Math.PI; //inches
  private static final double WHEEL_ROTATIONS_PER_PANEL_ROTATION 
    = PANEL_CIRCUMFERENCE / ROTATOR_WHEEL_CIRCUMFERENCE;
  private static final double TARGET_WHEEL_ROTATIONS 
    = WHEEL_ROTATIONS_PER_PANEL_ROTATION * TARGET_PANEL_ROTATIONS;
  private static final double TARGET_WHEEL_COUNT
    = TARGET_WHEEL_ROTATIONS * COUNTS_PER_WHEEL_ROTATION;

  private Control_Panel cp;
  /**
   * Creates a new SimpRotateControl.
   */
  public SimpRotateControl(Control_Panel controlPanel) {
    // Use addRequirements() here to declare subsystem dependencies.
    cp = controlPanel;
    addRequirements(cp);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cp.resetEncoder();
    cp.extendArm();
    //TODO: See if we need to delay starting the motor after extending
    cp.setSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cp.setSpeed(0);
    cp.retractArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cp.getDistance() >= TARGET_WHEEL_COUNT;
  }
}
