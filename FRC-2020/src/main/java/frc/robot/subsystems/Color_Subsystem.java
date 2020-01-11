/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class Color_Subsystem extends SubsystemBase {
  /**
   * Creates a new Color_Subsystem.
   */

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor;
  private ColorMatchResult match;
  private int proximity;

  public Color_Subsystem() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateColorSensor(){
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);
    proximity = m_colorSensor.getProximity();
  }


  public String getColor(){
    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    return colorString;
  }

  public double[] getRgb(){
      return new double[] {detectedColor.red, detectedColor.green, detectedColor.blue};
  }

  public int getProximity(){
    return proximity;
  }

  public void printLog(){
    double[] rgb = getRgb();

    SmartDashboard.putString("Color Match", getColor());
    SmartDashboard.putNumber("Color Confidence", match.confidence);
    SmartDashboard.putNumber("Red", rgb[0]);
    SmartDashboard.putNumber("Green", rgb[1]);
    SmartDashboard.putNumber("Blue", rgb[2]);
    SmartDashboard.putNumber("Color Sensor Proximity", getProximity());
  }


}
