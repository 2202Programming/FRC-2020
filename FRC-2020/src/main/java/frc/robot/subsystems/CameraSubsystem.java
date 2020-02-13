package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

  private UsbCamera frontCamera;
  private UsbCamera rearCamera;
  //private UsbCamera armCamera;
  private VideoSink switchedCamera;
  private Integer currentCamera = 1;

  public CameraSubsystem() {

    //NOTE: There is a bug in CameraServer where cameras constructed before the first  getInstance() call aren't published,
    //      so it's VERY IMPORTANT to call getInstance() prior to constructing the first camera
    //      https://www.chiefdelphi.com/t/stream-from-jetson-to-rio/343525/2
    //CameraServer cs = CameraServer.getInstance();

    frontCamera = CameraServer.getInstance().startAutomaticCapture("Front Drive", Constants.FRONT_DRIVE_CAMERA_PATH);
    frontCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 20);
    
    rearCamera = CameraServer.getInstance().startAutomaticCapture("Rear Drive", Constants.REAR_DRIVE_CAMERA_PATH);
    rearCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 20);

    //armCamera = CameraServer.getInstance().startAutomaticCapture("Arm", RobotMap.ARM_CAMERA_PATH);
    //armCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 240, 240, 15);

    switchedCamera = CameraServer.getInstance().addSwitchedCamera("Switched Camera");
    
    //frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    switchedCamera.setSource(frontCamera);
  }

  public void toggleDriveCamera() { //switch drive camera to other USB webcam
    if (currentCamera < 0) {
      switchedCamera.setSource(frontCamera);
      System.out.println("Setting Front Camera");
      currentCamera = 1;
    }
    else {
      switchedCamera.setSource(rearCamera);
      System.out.println("Setting Rear Camera");
      currentCamera = -1;
    }
  }
}