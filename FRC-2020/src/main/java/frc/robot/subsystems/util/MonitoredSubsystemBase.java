// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class MonitoredSubsystemBase extends SubsystemBase {
  static final double k_uS_to_mS = 1000.0; // microseconds
  
  static HashMap<String, MonitoredSubsystemBase> s_map = new HashMap<>();
  static boolean s_enabled = true;

  long min;
  long max;
  long total;
  long count;

  public MonitoredSubsystemBase() {
    super();
    s_map.put(getName(), this);
    reset();
  }

  public abstract void monitored_periodic();

  @Override
  public void periodic() {
    if (!s_enabled) {
      monitored_periodic();
      return;
    }
    var startTime = RobotController.getFPGATime();
    count++;
    monitored_periodic();
    var time = RobotController.getFPGATime() - startTime;
    //save the stats
    total += time;
    min = Math.min(min, time);
    max = Math.max(max, time);
  }

  public void reset() {
    count = max = total = 0L;
    min = 999999999L;
  }

  public double getMinTime() {
    return this.min / k_uS_to_mS;
  } 

  public double getMaxTime() {
    return this.max / k_uS_to_mS;
  } 

  public double getAvgTime() {
    return (count > 0) ? (this.total /this.count) / k_uS_to_mS  : -1.0;
  } 

  public String toString() {
   return String.format("Timing for %-25.20s: min = %05.5fms  max = %05.5fms avg = %05.5fms (%d) \n", getName(),
    getMinTime(), getMaxTime(), getAvgTime(), count );
  }

  public static String getStats() {
    StringBuilder s = new StringBuilder();

    s.append("\n***********************************************************\n");
    for (String k : s_map.keySet()) { 
      s.append(s_map.get(k).toString());
    }
    s.append("***********************************************************\n");
    return s.toString();
  }

  public static void resetAllStats() {
    for (String k : s_map.keySet()) { 
     s_map.get(k).reset();
    }
  }

  public static void setEnabled(boolean enabled) {
    s_enabled = enabled;
  }

}
