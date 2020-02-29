/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.panel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Control_Panel;

public class SimpRotateControl extends CommandBase {

  private static final double WHEEL_SPEED = 0.7;

  private static final double COUNTS_PER_WHEEL_ROTATION = 4096;

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
   * Rotates the control panel between 3 and 5 full revolutions.
   */
  public SimpRotateControl(Control_Panel controlPanel) {
    cp = controlPanel;
    addRequirements(cp);
  } 

  // Start spinning
  @Override
  public void initialize() {
    cp.resetEncoder();
    //cp.extendArm();
    //TODO: See if we need to delay starting the motor after extending
    cp.setSpeed(WHEEL_SPEED);
  }

  @Override
  public void execute() {
  }

  // Stop spinning when we're done
  @Override
  public void end(boolean interrupted) {
    cp.setSpeed(0);
    //cp.retractArm();
  }

  // Check if we've reached the desired number of counts on the wheel's encoder
  @Override
  public boolean isFinished() {
    return cp.getDistance() >= TARGET_WHEEL_COUNT;
  }
}
