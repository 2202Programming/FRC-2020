/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import static frc.robot.Constants.DT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.FlywheelRPM;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Magazine_Subsystem;

public class ShooterOn extends CommandBase {
  private double HIGH_ANGLE = 40;

  private double SLOW_MAG_REVERSE = -0.8; // motor power
  private double FAST_MAG_FORWARD =  1; // motor power
  
  Intake_Subsystem intake;
  Magazine_Subsystem magazine;

  private final int backupCount;
  private int count;

  // Speed to use speed to use based on high/low mag position, distance or other function
  // set by call to calculateShooterSpeed().
  FlywheelRPM rpmSetpoint = new FlywheelRPM(0, 0);
  
  // Flywheel speed calculated from the vel/rps targets
  FlywheelRPM lowFlywheelGoal;
  FlywheelRPM highFlywheelGoal;

  // testing/reporting controls
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
  int atGoalCount = 0;       // require flywheels to be at goal for a few frames

  /**
   * Constant/control data needed as input for this command.
   */
  public static class Data {
    public ShooterSettings LowGoal;   
    public ShooterSettings HighGoal; 
    public double Tolerance;   // % of target goal ~1% to 2%
    public double BackupSec;   // seconds to backup
    public int    AtGoalBeforeShoot;      //

    public Data() {
      // do nothing
    }
    /**
     * copy constructor
     * @param d
     */
    public Data( Data d) {
      // make deep copy of incoming data
      LowGoal = new ShooterSettings(d.LowGoal);
      HighGoal = new ShooterSettings(d.HighGoal);
      BackupSec = d.BackupSec;
      Tolerance = d.Tolerance;
      AtGoalBeforeShoot = d.AtGoalBeforeShoot;
    }
  }

  // commanded target goals
  Data goals;

  public ShooterOn(Intake_Subsystem intake) {
    // using dummy upper/lower target rpm.  If this construsctor is used, calculateShooterSpeed()
    // will be overriden.  USe the Constant data.
    this(intake, ShooterOnCmd.data);      
  }

  public ShooterOn(Intake_Subsystem intake, ShooterOn.Data cmdData) {
    this.intake = intake;
    magazine = intake.getMagazine();
    goals = cmdData;
    backupCount = (int) Math.floor(goals.BackupSec / DT);
    stage = Stage.DoingNothing;

    //convert powercell velocity to RPM for high and low positions
    lowFlywheelGoal = intake.calculateGoals(cmdData.LowGoal);
    highFlywheelGoal = intake.calculateGoals(cmdData.HighGoal);

    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = Stage.DoingNothing;
    magazine.beltOff();
    intake.intakeOff();
    count = 0;
    atGoalCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    switch(stage){
      case DoingNothing:
        magazine.beltOn(SLOW_MAG_REVERSE);
        stage = Stage.BackingMagazine;
      break;

      case BackingMagazine:    
        //backup magazine for backup_count to get balls off flywheels
        if (count >= backupCount) {
          //done backing up, turn on flw wheels enter next stage
          magazine.beltOff();
          stage = Stage.WaitingForSolution;
        }
      break;

      case WaitingForSolution:
        if (!calculateShooterSpeed()) break;  // break means we don't have solution, keep waiting
        
        // we have our shoot target speeds, turn shooter on
        intake.shooterOn(rpmSetpoint);
        time = System.currentTimeMillis();    //time for spin up start
        stage = Stage.WaitingForFlyWheel;
      break;

      case WaitingForFlyWheel: //stage 1, pause magazine while shooters get to RPM goal
        if (intake.atGoalRPM(goals.Tolerance) && 
            (atGoalCount++ >= goals.AtGoalBeforeShoot)) {
          //Flywheel at speed, move to shooting
          stage = Stage.Shooting;
          magazine.beltOn(FAST_MAG_FORWARD);
        }
      break;

      case Shooting: 
        //magazine forward fast to shoot while at RPM goals              
        if (!intake.atGoalRPM(goals.Tolerance)) { 
          time = System.currentTimeMillis();
          atGoalCount = 0;
          //back to WaitingForFlywheel
          stage = Stage.WaitingForFlyWheel;
          magazine.beltOff();
       }
      break;
    }
    count++;
  }

  /**
   * calcuateShooterSpeed()
   *   Sets internal target speeds for shooter. 
   *   Override this to use ShooterOn command from different locations.
   * 
   *   Side effect: rpmSetpoint is loaded 
   * 
   * @return true if solution is found and we can shoot
   */
  public boolean calculateShooterSpeed(){
    if (magazine.getAngle() > HIGH_ANGLE) {
      // aiming high
      rpmSetpoint.copy(highFlywheelGoal);
    }
    else {
      // aiming low
      rpmSetpoint.copy(lowFlywheelGoal);
    }
    // nothing more to calculate or wait for, we have RPM solution
    return true;  
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage = Stage.DoingNothing;
    intake.shooterOff();
    magazine.beltOff();
    intake.intakeOff();
  }

}
