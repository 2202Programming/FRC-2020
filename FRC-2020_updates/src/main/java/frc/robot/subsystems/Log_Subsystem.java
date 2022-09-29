/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

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
 *   dpl    3/11/21   shut off logging by setting mod == 0, each system can have own rate now.
 * 
 */

public class Log_Subsystem extends MonitoredSubsystemBase {
  /**
   * Creates a new Log_Subsystem.
   */
  final Map<Logger, Integer> loggers = new HashMap<>();
  int counter;
  final int defaultMod;

  public Log_Subsystem(int defaultMod) {
    counter = 0;
    this.defaultMod = defaultMod;
  }

  /**
   * General Logger interface, takes anything that implements Logger interface
   */
  public synchronized void add(Logger ... devices) {
    for(Logger dev : devices)
    loggers.put(dev, defaultMod);
  }

  public void setLogModulo(Logger logger, int mod) {
    loggers.put(logger, mod);
  }

  public void log() {
    //SmartDashboard.putString("Command: ", Robot.command);
  }

  @Override
  public void monitored_periodic() {
    counter++;
    // call all log() if it's their time
    loggers.forEach(  (k, v) ->  {
      if ((v > 0) && (counter % v == 0) )
        k.log();
    });
  }
}
