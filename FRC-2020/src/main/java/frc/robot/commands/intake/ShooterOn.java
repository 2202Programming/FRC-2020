/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_Subsystem;

public class ShooterOn extends CommandBase {
  private double SLOW_MAG_REVERSE = -0.8; // motor power
  private double FAST_MAG_FORWARD =  1; // motor power
  
  Intake_Subsystem m_intake;
  private final double m_upperRpmTarget_low;
  private final double m_upperRpmTarget_high;
  private final double m_lowerRpmTarget_low;
  private final double m_lowerRpmTarget_high;
  private final int m_backupCount;
  private int m_count;

  // Speed to use speed to use based on high/low mag position, distance or other function
  // set by call to calculateShooterSpeed().
  double m_tolerance = 0.025;     //percent - RPM of flywheel
  double m_rpmUpper;  
  double m_rpmLower;  
  
  // testing/reporting controls
  int frameSkip = 1;        // used to skip N frames in Stage state machine
  double time;

  // Shooter states (verbs) for state machine
  enum Stage {
    DoingNothing,           // entry state, before button is pushed
    WaitingForSolution,     // calculating or moving for shot, see calcuateShooterSpeed()
    BackingMagazine,        // fixed time given on constructor
    WaitingForFlyWheel,     // holds until flywheel is at speed
    Shooting,               // mag running, balls flying, watching fw speeds 
  }
  Stage stage;

  public ShooterOn(Intake_Subsystem intake, double backupSec) {
    // using dummy upper/lower target rpm.  If this construsctor is used, calculateShooterSpeed()
    // will be overriden
    this(intake, 2000, 2000, 500, 500, backupSec);  
  }

  public ShooterOn(Intake_Subsystem intake, double upperRpmTarget_low, double upperRpmTarget_high, double lowerRpmTarget_low, double lowerRpmTarget_high, double backupSec) {
    m_intake = intake;
    m_upperRpmTarget_low = upperRpmTarget_low;
    m_upperRpmTarget_high = upperRpmTarget_high;
    m_lowerRpmTarget_low = lowerRpmTarget_low;
    m_lowerRpmTarget_high = lowerRpmTarget_high;
    m_backupCount = (int) Math.floor(backupSec / Constants.DT);

    stage = Stage.DoingNothing;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = Stage.DoingNothing;
    m_intake.magazineOff();
    m_intake.intakeOff();
    m_count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    switch(stage){
      case DoingNothing:
        m_intake.magazineOn(SLOW_MAG_REVERSE);
        stage = Stage.BackingMagazine;
      break;

      case BackingMagazine:    
        //backup magazine for backup_count to get balls off flywheels
        if (m_count >= m_backupCount) {
          //done backing up, turn on flw wheels enter next stage
          m_intake.magazineOff();
          stage = Stage.WaitingForSolution;
          
          // TODO: consider starting the flywheels with an estimate while we wait
          // for a computed solution.  In the fixed cases, this doesn't matter much. (one frame delay - 20ms)
        }
      break;

      case WaitingForSolution:
        if (!calculateShooterSpeed()) break;  // break means we don't have solution, keep waiting
        
        // we have our shoot target speeds, turn shooter on
        m_intake.shooterOn(m_rpmUpper, m_rpmLower);
        time = System.currentTimeMillis();    //time for spin up start
        stage =Stage.WaitingForFlyWheel;
        m_count = frameSkip - 1;              // force no delay on checking FW speed next pass
      break;

      case WaitingForFlyWheel: //stage 1, pause magazine while shooters get to RPM goal
        if (m_count % frameSkip == 0) {
          if (m_intake.atGoalRPM(m_tolerance)){
            String output = "*** MAG RESUME - Shooter Ramp-up Delay: " + (System.currentTimeMillis() - time) + " ms. \n" +
                  "Upper Goal: " + m_intake.getUpperTargetRPM() + ", Achieved Upper RPM: " + m_intake.getUpperRPM() + "\n" +
                  "Lower Goal: " + m_intake.getLowerTargetRPM() + ", Achieved Lower RPM: " + m_intake.getLowerRPM() + "\n\n";
            System.out.println(output);
            //Flywheel at speed, move to shooting
            stage = Stage.Shooting;
            m_intake.magazineOn(FAST_MAG_FORWARD);
          }
          //System.out.println("Upper Motor RPM:" + m_intake.upperRPM +", Lower: "+m_intake.lowerRPM +"\n");
         }
      break;

      case Shooting: 
        //magazine forward fast to shoot while at RPM goals
        if (m_count % frameSkip == 0) {                   
          if (!m_intake.atGoalRPM(m_tolerance)) { //back to WaitingForFlywheel if RPMs fall below tolerance
            time = System.currentTimeMillis();
            // hold off mag until flywheel is at target
            stage = Stage.WaitingForFlyWheel;
            m_intake.magazineOff();
            System.out.println("**MAG PAUSE - Upper RPM: " + m_intake.getUpperRPM() + ", Lower RPM: " + m_intake.getLowerRPM() + "\n");
         }
        }
      break;
    }
    m_count++;   
  }

  /**
   * calcuateShooterSpeed()
   *   Sets internal target speeds for shooter. 
   *   Override this to use ShooterOn command from different locations.
   * 
   *   sets m_rpmUpper and m_rpmLower for flywheel speeds.
   * 
   * @return true if solution is found and we can shoot
   */
  public boolean calculateShooterSpeed(){
    if (m_intake.isMagazineUp()) {
      // aiming high
      m_rpmUpper = m_upperRpmTarget_high;
      m_rpmLower = m_lowerRpmTarget_high;
    }
    else {
      // aiming low
      m_rpmUpper = m_upperRpmTarget_low;
      m_rpmLower = m_lowerRpmTarget_low;
    }
    // nothing more to calculate or wait for, we have RPM solution
    return true;  
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage = Stage.DoingNothing;
    m_intake.shooterOff();
    m_intake.magazineOff();
    m_intake.intakeOff();
  }

}
