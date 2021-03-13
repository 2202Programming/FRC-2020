package frc.robot.commands.generic;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.subsystems.ifx.Odometry;


public class PositionRecorder extends CommandBase {
  /**
   * Creates a new PositionRecorder.  Adapted from ligerbots!
   * 
   * 
   * Looks at the followTrajectory command, if running and samples the requested trajectory
   * 
   */
  public boolean isRunning = false;

  Odometry drivetrain;
  PrintWriter writer;
  long start;

  String directoryName="recordings";
  String workingDir;

  NetworkTableEntry directoryNameEntry;
  NetworkTableEntry isRunningEntry;
  
  public PositionRecorder(Odometry drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    initSmartDashboard();
    this.drivetrain = drivetrain;
  }
  
  public void initSmartDashboard(){
    ShuffleboardTab tab =Shuffleboard.getTab("Position Recorder");
    isRunningEntry=tab.add("Is running", isRunning).withWidget("Toggle Button").getEntry();
    directoryNameEntry=tab.add("Directory Name", directoryName).getEntry();

    isRunningEntry.addListener((EntryNotification e)-> setIsRunning(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);

    directoryNameEntry.addListener((EntryNotification e)-> directoryName=e.value.getString(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
  }

  public void setIsRunning(boolean running){
    System.out.println("Set is running: "+running);
    if(running!=isRunning){
      if(running){
        this.schedule();
      }else{
        this.cancel();
      }
    }
    if(isRunningEntry.getBoolean(false)!=running){
      isRunningEntry.setBoolean(running);
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRunning = true;
    writer=null;
    System.out.println("Position Recorder Begin");
    try {
      String filename = new SimpleDateFormat("MM-dd_HH_mm_ss").format(new Date())+".csv";
  
      workingDir =  Filesystem.getOperatingDirectory().toString() + "/" +directoryName;
      new File(workingDir).mkdirs();

      File f = new File(workingDir, filename);
      f.createNewFile();
      writer = new PrintWriter(f); // PrintWriter is buffered
      writer.println("elapsed-uS, x, y, rotation, traj-time, traj-x, traj-y, traj-rot");

      start= RobotController.getFPGATime(); 

      System.out.println("Writing to: " + f.getAbsolutePath());

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(writer==null)return;
    Pose2d currentPosition=drivetrain.getPose();
    try{
      long now = RobotController.getFPGATime();
      writer.print((now - start)/1000000.0 + ","
          +currentPosition.getX() + ","
          +currentPosition.getY() + ","
          +currentPosition.getRotation().getRadians());

      // log any trajectory we may be running
      Trajectory traj = followTrajectory.getCurrentTrajectory();
      if (traj != null) {
        long traj_start = followTrajectory.getStartTime();
        double traj_time = (now - traj_start)/1000000.0;
        Pose2d traj_pose = traj.sample(traj_time).poseMeters;
        writer.println("," + traj_time + ","
          +traj_pose.getX()+","
          +traj_pose.getY()+","
          +traj_pose.getRotation().getRadians() );
      }
      else {
        // nothing running - zeros
        writer.println(", 0.0, 0.0, 0.0, 0.0");
      }

    }catch(Exception e){
      e.printStackTrace();
      writer.close();
      writer=null;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isRunning=false;

    if(writer != null){
      writer.close();
      writer = null;
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {return true;}
}