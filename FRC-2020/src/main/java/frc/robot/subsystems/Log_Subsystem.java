/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.ArrayList;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;

/**
 * 
 * Log_Subsystem - calls log() functions for robot
 *     Calls one system every N logFrame counts.
 * 
 *   dpl    2/21/20   decoupled from specific subsytems, use anywhere 
 *                    you need a Logger.
 * 
 */

public class Log_Subsystem extends MonitoredSubsystemBase implements Logger {
  /**
   * Creates a new Log_Subsystem.
   */
  ArrayList<Logger> loggers = new ArrayList<Logger>();
  private int counter;
  private final int logFrame;    //when to call a Logger's log
  private int lastLog;

  public Log_Subsystem(int logFrameCount){
    counter = 0;
    this.logFrame = logFrameCount;
    this.lastLog = 0;
    //we are a Logger, so add us as first in list
    add(this);
  }

  /**
   * General Logger interface, takes anything that implements Logger interface
   */
  public synchronized void add(Logger ... devices) {
    for(Logger dev : devices)
    loggers.add(dev);
  }

  public void log() {
    //SmartDashboard.putString("Command: ", Robot.command);
  }

  @Override
  public void monitored_periodic() {
      if ((counter++ % logFrame) == 0) {
      loggers.get(lastLog++).log();
      
      //reset the lastLog when at the end of array
      lastLog = (lastLog >= loggers.size()) ? 0 : lastLog;
    }
  }
}
