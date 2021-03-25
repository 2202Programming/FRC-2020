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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
  public boolean convertToTrajectory = false;

  Odometry drivetrain;
  PrintWriter writer;
  long start=0;
  long traj_start=0;

  String directoryName="recordings";
  String workingDir;
  final Pose2d zeroPose = new Pose2d();

  ///.NetworkTableEntry directoryNameEntry;
  NetworkTableEntry NTE_isCaptureRunning;
  NetworkTableEntry NTE_convertToTrajectory;

  final RecordLine m_record;

  File recordFile;

  //helper class
  public static class RecordLine {
    /* public enum Data {
      time(0), poseX, poseY, poseRot,
      meas_ws_l, meas_ws_r, 
      cmd_ws_l, cmd_ws_r,
      traj_time, traj_poseX, traj_poseY, traj_poseRot
    } */

    public double time;
    public Pose2d robot_pose;
    public DifferentialDriveWheelSpeeds meas_speed;
    public DifferentialDriveWheelSpeeds cmd_speed;
    public double traj_time;
    public Pose2d traj_pose;

    public RecordLine() {
      time = 0.0;
      robot_pose = new Pose2d();
      traj_time = 0.0;
      traj_pose = new Pose2d();
      
      meas_speed = new DifferentialDriveWheelSpeeds();
      cmd_speed = new DifferentialDriveWheelSpeeds();
    }

    public RecordLine(double [] data) {
      time = data[0];
      robot_pose = new Pose2d(data[1], data[2], new Rotation2d(data[3]));
      meas_speed = new DifferentialDriveWheelSpeeds(data[4], data[5]);
      cmd_speed = new DifferentialDriveWheelSpeeds(data[6], data[7]);
      traj_time = data[8];
      traj_pose = new Pose2d(data[9], data[10], new Rotation2d(data[11]));
    }

    public static String toHeader() {
      return "elapsed-uS, x, y, rot-deg, " + 
          "meas-ws-left, meas-ws-right, cmd-ws-left, cmd-ws-right, "+
          "traj-time, traj-x, traj-y, traj-rot-deg,";

    }

    public static final String CSV_FORMAT = "%05.3f , %05.3f , %05.3f , %05.3f , " + 
                             "%05.3f , %05.3f , %05.3f , %05.3f , " + 
                             "%05.3f , %05.3f , %05.3f , %05.3f";

    public String toString() {
      return String.format( CSV_FORMAT,
          time,
          robot_pose.getX(),
          robot_pose.getY(),
          robot_pose.getRotation().getDegrees(),

          meas_speed.leftMetersPerSecond,
          meas_speed.rightMetersPerSecond,
          cmd_speed.leftMetersPerSecond,
          cmd_speed.rightMetersPerSecond,

          traj_time,
          traj_pose.getX(),
          traj_pose.getY(),
          traj_pose.getRotation().getDegrees()
        );
    }
  
    public static  RecordLine fromString(String line) {
      String[] tokens = line.split(",");
      double[] data = new double[tokens.length];
      for (int i=0; i < tokens.length ; i++ ) {
        data[i] = Double.parseDouble(tokens[i]);
      }
      return new RecordLine(data);
    }
  
  }


  public PositionRecorder(Odometry drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    initSmartDashboard();
    this.drivetrain = drivetrain;
    m_record = new RecordLine();

    // get structs for wheel speeds
    m_record.cmd_speed = drivetrain.getCommandedWheelSpeeds();
    m_record.meas_speed = drivetrain.getWheelSpeeds();

    setConvertWhenDone(false);
  }
  
  public void initSmartDashboard(){
    ShuffleboardTab tab =Shuffleboard.getTab("Position Recorder");
    NTE_isCaptureRunning=tab.add("Capture Running", isRunning).withWidget("Toggle Button").getEntry();    
    NTE_isCaptureRunning.addListener((EntryNotification e) 
        -> setIsRunning(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);

    NTE_convertToTrajectory=tab.add("Convert When Done", isRunning).withWidget("Toggle Button").getEntry();    
    NTE_convertToTrajectory.addListener((EntryNotification e) 
        -> setConvertWhenDone(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);


////directoryNameEntry=tab.add("Directory Name", directoryName).getEntry();
    ////directoryNameEntry.addListener((EntryNotification e)-> directoryName=e.value.getString(), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
  }

  public PositionRecorder setIsRunning(boolean running){
    System.out.println("Set is running: "+running);
    if(running!=isRunning){
      if(running){
        this.schedule();
      }else{
        this.cancel();
      }
    }
    if(NTE_isCaptureRunning.getBoolean(false)!=running){
      NTE_isCaptureRunning.setBoolean(running);
    }
    return this;
  }

  public PositionRecorder setConvertWhenDone(boolean convert) {
    convertToTrajectory = convert;
    
    // make sure nt reflects proper state
    if (NTE_convertToTrajectory.getBoolean(false) != convertToTrajectory)
      NTE_convertToTrajectory.setBoolean(convertToTrajectory);
    return this;
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

      recordFile = new File(workingDir, filename);
      recordFile.createNewFile();
      writer = new PrintWriter(recordFile);   // PrintWriter is buffered
      writer.println(RecordLine.toHeader());
      start= RobotController.getFPGATime();
      traj_start = 0;

      System.out.println("Recording to: " + recordFile.getAbsolutePath());

    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (writer==null) return;

    m_record.robot_pose = drivetrain.getPose();
    long now = RobotController.getFPGATime();
    m_record.time = (now - start)/1000000.0;

    // log any trajectory we may be running
    Trajectory traj = followTrajectory.getCurrentTrajectory();
    if (traj != null) {
        //save the trajectory starting time 
        if (traj_start ==0) traj_start = followTrajectory.getStartTime();
        m_record.traj_time = (now - traj_start)/1000000.0;
        m_record.traj_pose = traj.sample(m_record.traj_time).poseMeters;    
    }
    else {
      m_record.traj_time = 0.0;
      m_record.traj_pose = zeroPose;
    }
    // write it out
    try {
      writer.println(m_record.toString());
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
    if (convertToTrajectory) {
      var converter = new ConvertRecordingToTrajectory();
      converter.inputFile(recordFile);
      converter.saveTrajectory();
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