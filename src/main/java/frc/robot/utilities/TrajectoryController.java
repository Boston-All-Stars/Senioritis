package frc.robot.utilities;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.Gains;
import frc.robot.Robot;
import java.util.HashMap;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {

  public enum Autos {
    // Blue Paths
    SAMPLE_BLUE("simpleblue", 4, 1),

    // Red Paths
    // to make red paths, take equivalent paths and replace every Y value with 8.01 - Y
    // Then adjust headings as necessary for a smooth trajectory
    // It won't look right, but, well, it is what it is
    SAMPLE_RED("simplered", 4, 1);
    private PathPlannerTrajectory trajectory;

    private Autos(String filename, double maxVel, double maxAccel) {
      trajectory = PathPlanner.loadPath(filename, new PathConstraints(maxVel, maxAccel));
    }

    private Autos(String filename) {
      this(
          filename,
          Constants.DriveConstants.MAX_SWERVE_VEL,
          Constants.DriveConstants.MAX_SWERVE_ACCEL);
    }

    private void clear() {
      this.trajectory = null;
    }

    public static void clearAll() {
      for (Autos auto : Autos.values()) {
        auto.clear();
      }
    }

    public PathPlannerTrajectory getTrajectory() {
      return trajectory;
    }
  }

  private static TrajectoryController instance;
  Timer timer = new Timer();
  PathPlannerTrajectory traj;
  HashMap<String, Command> eventMap = new HashMap<>();
  PathPlannerState targetState;
  PPHolonomicDriveController controller =
      new PPHolonomicDriveController(
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_X.createWpilibController(),
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_Y.createWpilibController(),
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION.createWpilibController());

  private TrajectoryController() {}

  public static TrajectoryController getInstance() {
    if (instance == null) {
      instance = new TrajectoryController();
    }
    return instance;
  }

  public void changePath(@NonNull PathPlannerTrajectory newTrajectory) {
    traj = newTrajectory;

    Logger.getInstance().recordOutput("Trajectory/Trajectory Obj", newTrajectory);
    Logger.getInstance().recordOutput("Trajectory/Total time", newTrajectory.getTotalTimeSeconds());
    Logger.getInstance()
        .recordOutput(
            "Trajectory/Initial State Velocity",
            newTrajectory.getInitialState().velocityMetersPerSecond);

    timer.reset();
    timer.stop();
  }

  public boolean isFinished() {
    return timer.get() >= traj.getTotalTimeSeconds();
  }

  public ChassisSpeeds update() {
    if (traj == null) {
      return new ChassisSpeeds();
    }
    if (timer.get() == 0) {
      timer.start();
    }

    if (isFinished()) {
      targetState = traj.getEndState();
    } else {
      targetState = (PathPlannerState) traj.sample(timer.get());
    }

    Logger.getInstance()
        .recordOutput(
            "Trajectory/Target Pose",
            new double[] {
              targetState.poseMeters.getX(),
              targetState.poseMeters.getY(),
              targetState.holonomicRotation.getDegrees()
            });
    Logger.getInstance().recordOutput("Trajectory/timer", timer.get());
    if (!isFinished()) {
      var speeds = controller.calculate(Robot.swerveDrive.getPose(), targetState);
      return speeds;
    } else return new ChassisSpeeds();
  }
}
