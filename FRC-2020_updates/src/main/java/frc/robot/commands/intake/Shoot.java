package frc.robot.commands.intake;

import static frc.robot.Constants.DT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Magazine_Subsystem;

public class Shoot extends CommandBase {
  //belt constants
  double SLOW_MAG_REVERSE = -0.8; // motor power
  double FAST_MAG_FORWARD =  1;   // motor power

  // shooting controll constants
  final double BACKUPSEC = 0.1;     // secs to back up before shooting
  final int AtGoalBeforeShoot = 10;
  
  final Intake_Subsystem intake;
  final Magazine_Subsystem magazine;
  int backupCount;
  int count;

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
  
  ShooterSettings shooterSettings;  // pc vel, rotation, and mag angle

  /**
   * Shoot where we are at with rea
   * 
   * @param intake
   */
  public Shoot() {
    this.intake = RobotContainer.getInstance().intake;
    this.magazine = intake.getMagazine();
    this.backupCount = (int) Math.floor(BACKUPSEC / DT); 
    this.stage = Stage.DoingNothing;
    this.shooterSettings = null;     // use what the intake has

    // we will control the belt for shooting, add it as requirement
    addRequirements(magazine);
  }

  /**
   * Shoot at a specific command data set
   * @param intake
   * @param cmdData
   */
  public Shoot(ShooterSettings ss) {
    this();
    this.shooterSettings = ss;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = Stage.DoingNothing;
    magazine.beltOff();
    intake.intakeOff();
    if (shooterSettings != null)  {
      // use what this command was given
      intake.setShooterSettings(shooterSettings);
    }
    count = 0;
    atGoalCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // state machine for controlling belt and flywheels
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
        if (!calculateShooterSettings())  //possible override of calculateShooterSettings()
          break;  // break means we don't have solution, keep waiting
        
        // we have our shoot target speeds, turn shooter on
        intake.spinupShooter();
        stage = Stage.WaitingForFlyWheel;
      break;

      case WaitingForFlyWheel: //stage 1, pause magazine while shooters get to RPM goal
        if (intake.isReadyToShoot() && 
            (atGoalCount++ >= AtGoalBeforeShoot)) {
          //Flywheel at speed, move to shooting
          stage = Stage.Shooting;
          magazine.beltOn(FAST_MAG_FORWARD);
         
        }
      break;

      case Shooting: 
        //magazine forward fast to shoot while at RPM goals              
        if (!intake.isReadyToShoot()) { 
          atGoalCount = 0;
          //back to WaitingForFlywheel
          stage = Stage.WaitingForFlyWheel;
          magazine.beltOff(); 
          magazine.removePC();   // we shot 1
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
   *   Side effect: changes goals.ShooterSetting if needed
   *   when overriden by a more complex Shoot() command 
   * 
   * @return true if solution is found and we can shoot
   */
  public boolean calculateShooterSettings() {
    // We have them by defaule in goals.ShooterSetting
    // nothing more to calculate or wait for, we have RPM/angle solution
    return true;  
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stage = Stage.DoingNothing;
    intake.shooterOff();
    magazine.beltOff();
    intake.intakeOff();
    magazine.setPC(0);
  }

  @Override
  public boolean isFinished() {
    // done when nothing else to shoot
    //return (magazine.getPC() == 0);
    return false;
  }

}
