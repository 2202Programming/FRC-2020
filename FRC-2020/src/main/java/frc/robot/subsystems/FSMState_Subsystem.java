/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FSMState_Subsystem extends SubsystemBase {
  private static String gameData;
  /**
   * Creates a new FSMState_Subsystem.
   */
  public FSMState_Subsystem() {
    gameData = "";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getGameData()
  {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          gameData = "Blue";
          break;
        case 'G' :
          //Green case code
          gameData = "Green";
          break;
        case 'R' :
          //Red case code
          gameData = "Red";
          break;
        case 'Y' :
          //Yellow case code
          gameData = "Yellow";
          break;
        default :
          //This is corrupt data
          gameData = "";
          break;
      }
    } else {
      //Code for no data received yet
      gameData = "";
    }
  }

  public static String getColor()
  {
      return gameData;
  }

  public void print()
  {
      SmartDashboard.putString("Position Control Color", gameData);
  }
}
