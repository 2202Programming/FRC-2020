package frc.robot.commands.test.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

public class CreateCircle {
  final double dt = .1;  // sec timestep
  final double radius;
  final double k;
  final double velocity;
  final double distance;
  final double totalTime;
  final double w;

  double X0;
  double Y0;
  double theta0;

  Trajectory trajectory;
  Rotation2d robot_offset;
  //Pose2d initPose;
  //Pose2d finalPose;

  public CreateCircle(double radius, double velocity, double degrees) {
    // inital position of robot... [0,0,90] [x, y, theta]
    
    this.velocity = velocity;

    ArrayList<State> circlePoints = new ArrayList<>();
    distance =  Math.abs(Math.toRadians(degrees) * radius);
    totalTime =  distance / velocity; // seconds
    w = Math.toRadians(degrees) / totalTime; // radians /s
    k = Math.copySign( 1.0 / radius, degrees);

    // field coordinates
    X0 = 0.0; 
    theta0 = 0.0;  // portion of the arc we are computing,  [0 .. degrees] or [180 .. deg]
    this.radius = radius;

    // positive to the left, negitive to the right; adjust so robot is 0, 0
    // also adjust radius based on left/right curvature
    if (k < 0.0) {
      robot_offset = Rotation2d.fromDegrees(180);  //assume robot is facing away from us
      Y0 = radius;
      theta0 = Math.PI;
    } 
    else { //positive curvature
      robot_offset = Rotation2d.fromDegrees(0);  //assume robot is facing away from us
      Y0 = -radius;
      theta0=0; 
    } 

    // create a short line in X direction 
    var line = lineSegment(0.1);  //was "radius"
    var states = line.getStates();
    int count = states.size() / 2;

    for (int i=0; i < count; i++) {
      circlePoints.add(states.get(i));
    }
    State lineOffset = states.get(count + 1);

    // walk the circle - field coord, theta is not robot, it is just for calc X, Y of arc 
    for (double t = 0.0; t < (totalTime); t += dt) {
      circlePoints.add(calculatePoint(t, lineOffset));
    }
    //add one more point at the exact ending time
    circlePoints.add(calculatePoint(totalTime, lineOffset));
    var lastCircleState = circlePoints.get(circlePoints.size()-1);

    // now add the remaining line
    for(int i= count+2; i < states.size(); i++) {
      var linePt = states.get(i);

      var p = new Pose2d(linePt.poseMeters.getTranslation().plus(lastCircleState.poseMeters.getTranslation())
                                .minus(lineOffset.poseMeters.getTranslation()),
                         lastCircleState.poseMeters.getRotation() );

      var st = new State(linePt.timeSeconds - lineOffset.timeSeconds + lastCircleState.timeSeconds, 
                linePt.velocityMetersPerSecond,
                linePt.accelerationMetersPerSecondSq,
                p,
                linePt.curvatureRadPerMeter);
      circlePoints.add(st);
    }

    // construct the trajectory
    trajectory = new Trajectory(circlePoints);
  }

  /**
   * 
   * @param t - where in the time series to calculate based on circle
   * @return  State object on the circle
   */
  State calculatePoint(double t, State offset) {
    double theta = w * t + theta0;
    Rotation2d thetaRot2d = new Rotation2d(theta);
    double y = -(radius * thetaRot2d.getCos() + Y0);
    double x = radius * thetaRot2d.getSin() + X0;

    // correct theta for robot pose
    var th = thetaRot2d.minus(robot_offset); 
    ///debug var th_deg = th.getDegrees();
    var pose = new Pose2d(x + offset.poseMeters.getX(), 
                          y + offset.poseMeters.getY(), th);
    return new State(offset.timeSeconds + t, velocity, 0.0, pose, k);
  }

  public Trajectory getTrajectory() {return trajectory;}

  public String toString() {
    return 
    "Circle created: \n" +
      "totalTime = " +  trajectory.getTotalTimeSeconds() +"\n" +
      "point count = " +  trajectory.getStates().size() +"\n" +
      "sample(0) = " +  trajectory.sample(0) + "\n" +
      "sample(T) = " +  trajectory.sample(trajectory.getTotalTimeSeconds()) + "\n" +
      "\n";
  }

  public static void main(String... args) {
    CreateCircle testCircle = new CreateCircle(5, 1.0, 180);
    System.out.println(testCircle);

    CreateCircle testCircle2 = new CreateCircle(5, 1.0, -180);
    System.out.println(testCircle2);
  }

  Trajectory lineSegment(double distance) {
    TrajectoryConfig config =
    new TrajectoryConfig(velocity, 2*velocity)  // assume we can get to target speed in .5 seconds
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.RamseteProfile.kDriveKinematics);
        
    Trajectory segment = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0) ),
        //new Pose2d(startPose.getX(),startPose.getY(),new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(distance / 2.0, 0.0)
        ),
        //new Pose2d(endPose.getX(),endPose.getY(),new Rotation2d(0)),
        new Pose2d(distance, 0.0, new Rotation2d(0)),
        // Pass config
        config ); 
        return segment;
  }

}