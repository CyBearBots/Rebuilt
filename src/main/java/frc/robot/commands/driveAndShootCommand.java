package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;

public class driveAndShootCommand extends Command {
  
  private static final double kP_TRANSLATE        = 2.0;
  private static final double kP_ROTATE           = 0.075;
  private static final double kD_ROTATE           = 0.004;
  private static final double MAX_DRIVE_SPEED     = 0.4;
  private static final double MAX_TURN_SPEED      = 0.4;
  private static final double TRANSLATE_TOLERANCE = 0.05;
  private static final double ROTATE_TOLERANCE    = 2.0;
  private static final double STOP_DISTANCE       = 1.2;

  private int tagId1;
  private int tagId2;

  private static final int BLUE_TAG_1 = 25;
  private static final int BLUE_TAG_2 = 26;
  private static final int RED_TAG_1  = 9;
  private static final int RED_TAG_2  = 10;

  private final SwerveDrive      swerveDrive;
  private final Vision           vision;
  private final ShooterSubsystem shooter;
  private final PIDController    rotatePID;
  private final PIDController    xPID;
  private final PIDController    yPID;

  private Pose2d  targetPose;
  private boolean tagConfirmed = false;
  private boolean aligned = false;
  private boolean shootingStarted = false;

  double distanceToHub;
  double topShootRPM;

  public driveAndShootCommand(SwerveDrive swerveDrive, Vision vision, ShooterSubsystem shooter) {
    this.swerveDrive = swerveDrive;
    this.vision      = vision;
    this.shooter     = shooter;

    rotatePID = new PIDController(kP_ROTATE, 0, kD_ROTATE);
    rotatePID.enableContinuousInput(-180.0, 180.0);
    rotatePID.setTolerance(ROTATE_TOLERANCE);

    xPID = new PIDController(kP_TRANSLATE, 0, 0);
    xPID.setTolerance(TRANSLATE_TOLERANCE);

    yPID = new PIDController(kP_TRANSLATE, 0, 0);
    yPID.setTolerance(TRANSLATE_TOLERANCE);

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    tagConfirmed = false;
    aligned = false;
    shootingStarted = false;

    rotatePID.reset();
    xPID.reset();
    yPID.reset();

    boolean isBlue = DriverStation.getAlliance()
        .map(a -> a == Alliance.Blue)
        .orElse(true);

    tagId1 = isBlue ? BLUE_TAG_1 : RED_TAG_1;
    tagId2 = isBlue ? BLUE_TAG_2 : RED_TAG_2;

    vision.updatePoseEstimation(swerveDrive);

    targetPose = swerveDrive.getPose(); // fallback
  }

  private boolean seesAnyTag() {
    return vision.getTargetFromId(tagId1, Vision.Cameras.CAMERA_1) != null ||
           vision.getTargetFromId(tagId1, Vision.Cameras.CAMERA_2) != null ||
           vision.getTargetFromId(tagId2, Vision.Cameras.CAMERA_1) != null ||
           vision.getTargetFromId(tagId2, Vision.Cameras.CAMERA_2) != null;
  }

  private void computeTargetPose() {
    Optional<Pose3d> tag1 = Vision.fieldLayout.getTagPose(tagId1);
    Optional<Pose3d> tag2 = Vision.fieldLayout.getTagPose(tagId2);

    Translation2d hubCenter;

    if (tag1.isPresent() && tag2.isPresent()) {
      Translation2d t1 = tag1.get().toPose2d().getTranslation();
      Translation2d t2 = tag2.get().toPose2d().getTranslation();
      hubCenter = new Translation2d(
          (t1.getX() + t2.getX()) / 2.0,
          (t1.getY() + t2.getY()) / 2.0);
    } else if (tag1.isPresent()) {
      hubCenter = tag1.get().toPose2d().getTranslation();
    } else if (tag2.isPresent()) {
      hubCenter = tag2.get().toPose2d().getTranslation();
    } else {
      return; // no valid tag
    }

    Pose2d currentPose = swerveDrive.getPose();
    Translation2d dir = hubCenter.minus(currentPose.getTranslation());

    double angle = Math.atan2(dir.getY(), dir.getX());

    Translation2d stopPoint = hubCenter.minus(
        new Translation2d(Math.cos(angle) * STOP_DISTANCE,
                          Math.sin(angle) * STOP_DISTANCE));

    targetPose = new Pose2d(stopPoint, new Rotation2d(angle));
  }

  @Override
  public void execute() {

    distanceToHub = shooter.getDistanceToHub(vision, tagId1, tagId2);
    topShootRPM = shooter.calculateRPMFromDistance(distanceToHub);

    vision.updatePoseEstimation(swerveDrive);

    if (!tagConfirmed) {
      if (seesAnyTag()) {
        tagConfirmed = true;
        computeTargetPose();
      } else {
        swerveDrive.drive(new Translation2d(0,0),0,true,false);
        return;
      }
    }

    Pose2d currentPose = swerveDrive.getPose();

    if (!aligned) {
      double xOutput = MathUtil.clamp(
          xPID.calculate(currentPose.getX(), targetPose.getX()),
          -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

      double yOutput = MathUtil.clamp(
          yPID.calculate(currentPose.getY(), targetPose.getY()),
          -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

      double rotOutput = MathUtil.clamp(
          rotatePID.calculate(currentPose.getRotation().getDegrees(),
                              targetPose.getRotation().getDegrees()),
          -MAX_TURN_SPEED, MAX_TURN_SPEED);

      swerveDrive.drive(new Translation2d(xOutput, yOutput), rotOutput, true, false);

      if (xPID.atSetpoint() && yPID.atSetpoint() && rotatePID.atSetpoint()) {
        aligned = true;
        swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
      }

    } else {
      swerveDrive.drive(new Translation2d(0, 0), 0, true, false);

      shooter.setTargetRPM(
          topShootRPM,
          topShootRPM * ShooterConstants.spinRatio);

      shootingStarted = true;
    }
  }

  @Override
  public boolean isFinished() {

    if (!tagConfirmed) return false;

    if (shootingStarted) {

      return shooter.atSetpoint(
          topShootRPM,
          topShootRPM * ShooterConstants.spinRatio,
          ShooterConstants.rpmTolerance);
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
    shooter.stop();
  }
}