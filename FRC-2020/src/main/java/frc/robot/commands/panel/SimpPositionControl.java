/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.panel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Control_Panel;

public class SimpPositionControl extends CommandBase {

  //Cannot spin too fast or we overshoot due to lack of friction or the color detector being unable to keep up
  private static final double WHEEL_SPEED = 0.25;
  private Control_Panel cp;
  private Color_Subsystem detector;

  private String targetColor;

  /**
   * Rotates the control panel so the field's color detector receives whatever is specified by FMS
   */
  public SimpPositionControl(Control_Panel cp, Color_Subsystem detector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cp = cp;
    this.detector = detector;
    addRequirements(cp, detector);

    targetColor = calcTargetColor(cp.getTargetColor());
  }

  // Start spinning
  @Override
  public void initialize() {
    //cp.extendArm();
    cp.setSpeed(WHEEL_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Stop spinning
  @Override
  public void end(boolean interrupted) {
    cp.setSpeed(0);
    //cp.retractArm();
  }

  // Check if our current color is the color we want
  @Override
  public boolean isFinished() {
    return detector.getColor().equals(targetColor);
  }


  /*
  Determines which color we should be detecting when the panel is rotated so the field's detector is on the right color.
  The field detector is perpendicular to where we spin and read color.
  (One the panel, Blue and Red are perpendicular, and Yellow and Green are perpendicular)
  */
  
  public String calcTargetColor(String fieldColor) {
    String target = "";
    switch (fieldColor) {
      case "Blue":
        target = "Red";
        break;
      case "Red":
        target = "Blue";
        break;
      case "Yellow":
        target = "Green";
        break;
      case "Green":
        target = "Yellow";
        break;
    }

    return target;
  }
}
