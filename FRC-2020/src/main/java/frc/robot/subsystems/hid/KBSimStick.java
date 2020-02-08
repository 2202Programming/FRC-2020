
package frc.robot.subsystems.hid;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.hal.HAL;

/**
 * 
 * 
 */

public class KBSimStick extends GenericHID {
 
    public KBSimStick(int port) {
    super(port);
  }

  public enum Button {
        //Buttons
        A1(3),A2(4),A3(5),
        B1(6),B2(7),B3(8),
        C1(9),C2(10),C3(11),
        FlapUp(1), FlapDown(2),
        Mode(30),  
        Start(12),
        Eject(13),
        
        //RED
        TriggerRed(14),PinkyTriggerRed(15),TopTriggerRed(18), PinkyTopTriggerRed(19),
        //GREEN
        TriggerGn(16), PinkyTriggerGn(17), TopTriggerGn(20), PinkyTopTriggerGn(21),
        ;
      public int button;
      private Button(int val) {button = val;}

    }

    
  @Override
  public double getX(Hand hand) {
    if (hand.value != Hand.kRight) {

    }
    return 0;
  }

  @Override
  public double getY(Hand hand) {
    // TODO Auto-generated method stub
    return 0;
  }
}