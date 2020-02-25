/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Add your docs here.
 */
public class SideboardController extends GenericHID {

    public SideboardController(int ID){
        super(ID);
    }

    @SuppressWarnings({ "MemberName", "PMD.SingularField" })
    public enum Button {
      // Buttons
      DelayA(1),DelayB(2),DelayC(3); 

      public int value;
      private Button(final int val) {
        value = val;
      }
    }

    @Override
    public double getX(Hand hand) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getY(Hand hand) {
        // TODO Auto-generated method stub
        return 0;
    }

}
