package frc.robot.util.misc;

import static frc.robot.commands.generic.ConvertRecordingToTrajectory.diff_modulo;

import org.junit.Before;
import org.junit.Test;

public class mod_diff_test {
  
  @Before
  public void setUp() {
  }

  @Test
  public void test_mod_diff() {

      //  a1 - a0 on +-180 CCW+
       assert diff_modulo(179, -179, 180) == -2;
       assert diff_modulo(-179, 179, 180) == +2;
       assert diff_modulo(1, -1, 180) == +2;
       assert diff_modulo(-1, 1, 180) == -2;
       assert diff_modulo(0, -90, 180) == +90;
       assert diff_modulo(0, 90, 180) == -90;
       assert diff_modulo(90, -90, 180) ==  +180;
       assert diff_modulo(-90, 90, 180) == -180;
      double pi2 = Math.PI/2;
      double sa = .01;
      double apos = pi2 - sa;
      double aneg = -apos;
      double sa2 = 2*sa;
      double tol = 1e-8;

      // check pi/2 radians against small angle  around zero and pi/2
      assert Math.abs(diff_modulo(apos, aneg, pi2) -  -sa2)  < tol ;   
      assert Math.abs(diff_modulo(aneg, apos, pi2) -   sa2)  < tol ;
      assert Math.abs(diff_modulo(sa, -sa, pi2)    -   sa2)  < tol ;
      assert Math.abs(diff_modulo(-sa, sa, pi2)    -  -sa2)  < tol ;
         
  }


}
