// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static frc.robot.Constants.DT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineAngle extends CommandBase {
  // manual up/down at const motor speed or to an exact position
  public enum Direction {
    Up, Down, ToPositon
  };

  static final double RPM = 3;
  static final double DEG_PER_SEC = 4.0;
  static final double DEG_PER_PERIOD = DEG_PER_SEC * DT;
  static final double CONFIRM_MOVE_DEG = 0.5;
  static final int BURP_COUNT = 5; // frames to let motor go down before unlocking

  final MagazinePositioner magPositioner;
  double cmd_degrees;
  Direction direction;
  ShooterSettings cmd_settings;

  // current state
  int m_delay;                // number of frames to try to unlock pawl, BURP_COUNT
  double m_init_angle;        // current angle at init of command
  double m_integrated_angle;  // counts up/down each frame if manual position
  boolean m_locked;           // lock stat at init of command
  boolean m_problem_detected; // couldn't unlock log and stop trying

  /** Creates a new MagazineAngle. */
  private  MagazineAngle(Intake_Subsystem intake){
    this.magPositioner = intake.getMagazine().getMagPositioner();
    this.cmd_settings = null;
    addRequirements(magPositioner);
  }

  public MagazineAngle(Intake_Subsystem intake, Direction dir) {
    this(intake);
    this.cmd_degrees = 0.0;
    this.direction = dir;
    this.cmd_settings = null;
  }

  public MagazineAngle(Intake_Subsystem intake, double degrees) {
    this(intake);
    this.cmd_degrees = degrees;
    this.direction = Direction.ToPositon;
  }

  public MagazineAngle(Intake_Subsystem intake, ShooterSettings settings) {
    this(intake);
    this.cmd_settings = settings;
    this.cmd_degrees = settings.angle;
    this.direction = Direction.ToPositon;
    addRequirements(magPositioner);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_problem_detected = false;
    m_init_angle = magPositioner.get();
    m_integrated_angle = m_init_angle;
    m_locked = magPositioner.isLocked();
    m_delay = 0;

    // see if we need to burp the motor to unlock the pawl
    if (m_locked
        && ((direction == Direction.Up) || ((direction == Direction.ToPositon) && (m_init_angle <= cmd_degrees)))) {
      // we have to back up the motor and unlock
      magPositioner.burp();
      magPositioner.unlock();
      m_delay = BURP_COUNT;
    } else {
      // heading down, confirm the unlock right away
      magPositioner.unlockConfirmed();
    }
  }

  @Override
  public void execute() {
    // we are waiting for motor to reverse a bit to off load pawl
    if ((m_delay-- >= 0) && 
        (magPositioner.isMovingDown() && 
        (m_init_angle - magPositioner.get()) > CONFIRM_MOVE_DEG)) {

      // moving down and far enough, unlock worked
      magPositioner.unlockConfirmed();
      m_delay = 0;
    } else {
      // still waiting for movement and within BURP_COUNT
      return;
    }

    if (magPositioner.getUnlockConfirmed() == false) {
      // we have a problem
      System.out.println("ERROR unlocking the pawl.  No mag angle motor commands issued.");
      m_problem_detected = true;
      return;
    }

    // getting here, now we know we are unlocked
    // up/down will ramp up a position, ToPosition just sets the end goal.
    switch (direction) {
      case Up:  //magPositioner.wind(RPM);
        m_integrated_angle += DEG_PER_PERIOD;
        break;
      case Down: //magPositioner.wind(-RPM);
        m_integrated_angle -= DEG_PER_PERIOD;
        break;
      case ToPositon:
        m_integrated_angle = cmd_degrees;
        break;
    }
    //
    magPositioner.setAngle(m_integrated_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // no matter how we got here, manual, or ToPosition, call it home.
    magPositioner.stopAndHold(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_problem_detected)    return true;
    if (direction == Direction.ToPositon)  return magPositioner.isAtSetpoint();
    return false; // manual never done
  }
}
