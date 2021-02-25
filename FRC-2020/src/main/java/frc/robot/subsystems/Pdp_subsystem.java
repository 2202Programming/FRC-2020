// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.ifx.Logger;

public class Pdp_subsystem extends SubsystemBase implements Logger {
  /** Creates a new Pdp_subsystem. */
  private final PowerDistributionPanel pdp;

  private NetworkTable table;
  private NetworkTableEntry nt_voltage;

  public Pdp_subsystem() {
    pdp = new PowerDistributionPanel(CAN.PDP);
    table = NetworkTableInstance.getDefault().getTable("PDP");
    nt_voltage = table.getEntry("Voltage/value");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    nt_voltage.setNumber(pdp.getVoltage());
  }

  public void log(){

    
  }
}
