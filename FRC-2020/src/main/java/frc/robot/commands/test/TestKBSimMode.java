/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hid.KBSimStick;
import frc.robot.subsystems.hid.KBSimStick.Button;

/**
 * Use during test mode to check modes
 */
public class TestKBSimMode {

    KBSimStick  stick;

    public  TestKBSimMode() {
        stick = new KBSimStick(1);
        stick.enableTriggerModeTrack();
    }

    public void periodic() {
        double x = stick.getAxis(KBSimStick.Axis.kX);
        double y = stick.getAxis(KBSimStick.Axis.kY);
        double th = stick.getAxis(KBSimStick.Axis.kThrottle);
        double rot = stick.getAxis(KBSimStick.Axis.kRot);

        boolean b = stick.getButton(Button.FlapUp);
        int bcount = stick.getButtonCount();

        int pov =  stick.getPOV();

        System.out.println("   throttle:"+ th +"  X:"+ x );
        System.out.println("   bcount:"+ bcount +"b:"+ b + " POV: " + pov );
        
    }
    public Command getCommand() {return null;}

}
