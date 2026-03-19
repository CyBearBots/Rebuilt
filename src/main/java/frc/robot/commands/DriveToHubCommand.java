package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;

public class DriveToHubCommand extends Command {

  private static final double kP_TRANSLATE   = 2.0;  // how fast it drives toward hub
  private static final double kP_ROTATE      = 0.075;
  private static final double kD_ROTATE      = 0.004;
  private static final double MAX_DRIVE_SPEED = 0.4;  // max translation speed (meters/s ish)
  private static final double MAX_TURN_SPEED  = 0.4;
  private static final double TRANSLATE_TOLERANCE = 0.05; // meters — close enough
  private static final double ROTATE_TOLERANCE    = 2.0;  // degrees — close enough

  private final SwerveDrive   swerveDrive;
  private final Vision        vision;
  private final PIDController rotatePID;
  private final PIDController xPID;
  private final PIDController yPID;

  private Pose2d targetPose; // where we want the robot to end up

  public DriveToHubCommand(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision      = vision;

    rotatePID = new PIDController(kP_ROTATE, 0, kD_ROTATE);
    rotatePID.enableContinuousInput(-180.0, 180.0);
    rotatePID.setTolerance(ROTATE_TOLERANCE);

    xPID = new PIDController(kP_TRANSLATE, 0, 0);
    xPID.setTolerance(TRANSLATE_TOLERANCE);

    yPID = new PIDController(kP_TRANSLATE, 0, 0);
    yPID.setTolerance(TRANSLATE_TOLERANCE);
  }

  @Override
  public void initialize() {
    boolean isBlue = DriverStation.getAlliance()
                                  .map(a -> a == Alliance.Blue)
                                  .orElse(true);

    // Pull hub AprilTag position straight from the field layout JSON
    // Blue hub = tag 7, Red hub = tag 4 — verify these IDs in your JSON
    Optional<Pose3d> hubTag = Vision.fieldLayout.getTagPose(isBlue ? 7 : 4);

    if (hubTag.isPresent()) {
      Translation2d hubTranslation = hubTag.get().toPose2d().getTranslation();

      // Face the hub — point the front of the robot toward it
      // The robot should end up AT the hub position facing it
      // You may want an offset here so you stop IN FRONT of the hub, not on top of it
      // e.g. stop 1.0 meter away — adjust as needed
      double offsetMeters = 1.0;
      Pose2d currentPose  = swerveDrive.getPose();
      Translation2d dir   = hubTranslation.minus(currentPose.getTranslation());
      double angle        = Math.atan2(dir.getY(), dir.getX());
      Translation2d stopPoint = hubTranslation.minus(
          new Translation2d(Math.cos(angle) * offsetMeters,
                            Math.sin(angle) * offsetMeters));

      targetPose = new Pose2d(stopPoint, new Rotation2d(angle));
    } else {
      // Tag not found, just stay put
      targetPose = swerveDrive.getPose();
    }

    rotatePID.reset();
    xPID.reset();
    yPID.reset();
  }

  @Override
  public void execute() {
    vision.updatePoseEstimation(swerveDrive);

    Pose2d currentPose = swerveDrive.getPose();

    // Translation output
    double xOutput = MathUtil.clamp(
        xPID.calculate(currentPose.getX(), targetPose.getX()),
        -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

    double yOutput = MathUtil.clamp(
        yPID.calculate(currentPose.getY(), targetPose.getY()),
        -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

    // Rotation output
    double rotOutput = MathUtil.clamp(
        rotatePID.calculate(currentPose.getRotation().getDegrees(),
                            targetPose.getRotation().getDegrees()),
        -MAX_TURN_SPEED, MAX_TURN_SPEED);

    swerveDrive.drive(
        new Translation2d(xOutput, yOutput),
        rotOutput,
        true,   // field-relative
        false   // open loop
    );
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && rotatePID.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
  }
}