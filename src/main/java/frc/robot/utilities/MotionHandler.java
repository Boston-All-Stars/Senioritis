package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class MotionHandler {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY,
    LOCKDOWN,
    SLOW_MODE,
    NULL
  }

  /**
   * Calculates SwerveModuleState objects using the heading controller.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveHeadingController() {

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * DriveConstants.MAX_SWERVE_VEL,
                ySpeed * DriveConstants.MAX_SWERVE_VEL,
                Units.degreesToRadians(HeadingController.getInstance().update()),
                Robot.swerveDrive.getYaw()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Calculates SwerveModuleState objects using pure driver control.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveFullControl() {
    double xSpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double ySpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double rSpeed =
        MathUtil.applyDeadband(Robot.driver.getRightX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * DriveConstants.MAX_SWERVE_VEL,
                ySpeed * DriveConstants.MAX_SWERVE_VEL,
                rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
                Rotation2d.fromDegrees(-Robot.swerveDrive.inputs.gyroYawPosition)));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Calculates swerveModuleStates using the current trajectory.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveTrajectory() {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(TrajectoryController.getInstance().update());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SWERVE_VEL);

    return swerveModuleStates;
  }

  /**
   * Sets the robot to an unmoving lockdown configuration which is difficult to push.
   *
   * @return The lockdown array of swerveModuleStates.
   */
  public static SwerveModuleState[] lockdown() {
    SwerveModuleState[] swerveModuleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45))
        };

    return swerveModuleStates;
  }

  public static SwerveModuleState[] driveSlow() {
    double xSpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double ySpeed =
        MathUtil.applyDeadband(Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double rSpeed =
        MathUtil.applyDeadband(Robot.driver.getRightX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE);
    double slowVel =
        DriveConstants.SLOW_MODE_REDUCTION * DriveConstants.MAX_SWERVE_VEL;

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * slowVel,
                ySpeed * slowVel,
                rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
                Rotation2d.fromDegrees(-Robot.swerveDrive.inputs.gyroYawPosition)));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, slowVel);

    return swerveModuleStates;
  }
}
