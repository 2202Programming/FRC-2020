// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.Logger;

public class Pdp_subsystem extends SubsystemBase implements Logger {
  /** Creates a new Pdp_subsystem. */

  private final PowerDistributionPanel pdp;
  private double[] channelCurrent;


  public Pdp_subsystem() {

    pdp = new PowerDistributionPanel(0);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void log(){
    for (int i = 0; i<16; i++){
      channelCurrent[i] = pdp.getCurrent(i);
      SmartDashboard.putNumber("/PDP/CurrentChannel"+i, channelCurrent[i]);
    }
    SmartDashboard.putNumber("/PDP/TotalCurrent", pdp.getTotalCurrent());
    SmartDashboard.putNumber("/PDP/Voltage", pdp.getVoltage());
    
  }
}
