/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Add your docs here.
 */
public class SideboardController extends GenericHID {
    
    public enum SBButton {
      // Buttons
      // functional names
      DelayA(1),DelayB(2),DelayC(3), 
      //Row1
      Sw11(1), Sw12(2), Sw13(3), Sw14(4), Sw15(5), Sw16(6),
      //Row2
      Sw21(7), Sw22(8), Sw23(9), Sw24(10), Sw25(11), Sw26(12);
      //Row3
      //cursed - Sw31(13), Sw32(14), Sw33(15), Sw34(16);
      
      @SuppressWarnings({ "MemberName", "PMD.SingularField" })
      public final int value;
      private SBButton(final int val) {
        value = val;
      }
    }

    public SideboardController(final int port){
     super(port);
     //hack
     HAL.report(tResourceType.kResourceType_XboxController, port + 1);
   }
  

    /** These functions are not supported by our sideboard but are required for GenericHID.
     *  so just retrun zero and log an error with the driverstation.
     */
    @Override
    public double getX(Hand hand) {
        // doesn't exist on sideboard, return zero
        //DriverStation.reportError("getX() called on Sideboard", false);
        return 0;
    }

    @Override
    public double getY(Hand hand) {
        // doesn't exist on sideboard, return zero
        //DriverStation.reportError("getY() called on Sideboard", false);
        return 0;
    }

}
