/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.misc;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.dummy.DummyVelocityDrive;
import frc.robot.subsystems.ifx.VelocityDrive;

/**
 * Add your docs here.
 */
public class xmlParserTest {

  static Map<String, Object> deviceMap = RobotContainer.deviceMap;
  VelocityDrive drive;

  @Before
  public void setUp() {
    drive = new DummyVelocityDrive();
    deviceMap.put("driveTrain", drive);
  }

  @Test
  public void testParser() {
        String filename = "NoPotato.xml";

        xmlParser myParser = new xmlParser();
        Command test = myParser.parse(filename);
        System.out.println("File parsed and returned: " + test.getName());
        
        assertEquals(filename, test.getName());
        assertTrue(test.getRequirements().contains(drive));

    }

}
