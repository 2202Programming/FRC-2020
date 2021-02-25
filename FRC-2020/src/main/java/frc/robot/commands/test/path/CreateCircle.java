package frc.robot.commands.test.path;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class CreateCircle {

  final double N = 10;  //points / second
  final double dt;  // sec timestep
  final double k;

  Trajectory trajectory;
  Pose2d initPose;
  Pose2d finalPose;

  public CreateCircle(double radius, double velocity, double degrees) {

    ArrayList<State> circlePoints = new ArrayList<>();
    double distance =  Math.abs(Math.toRadians(degrees) * radius);
    double totalTime =  distance / velocity; // seconds
    double w = Math.toRadians(degrees) / totalTime; // radians /s
    k = Math.copySign( 1.0 / radius, degrees);

    // set dt based on having N pts / sec
    dt = .1;

    // field coordinates// really function of incoming pose, assume [0,0,0] [x, y, theta]
    double X0 = -radius; 
    double Y0 = 0.0;
    double theta0 = 0.0;
    Rotation2d thetaRot2d;

    if (k < 0.0) {
        X0 = radius;
        radius *= -1.0;
      //theta0 = Math.toRadians(360);
    }

    // walk the circle
    for (double t = 0.0; t < (totalTime); t += dt) {
      double theta = w * t + theta0;
      thetaRot2d = new Rotation2d(theta);
      double x = radius * thetaRot2d.getCos() + X0;
      double y = radius * thetaRot2d.getSin() + Y0;

      var pose = new Pose2d(x, y, thetaRot2d);
      circlePoints.add(new State(t, velocity, 0.0, pose, k));
    }

    trajectory = new Trajectory(circlePoints);
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

}