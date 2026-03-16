package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;

/**
 * Rotates the robot in place so the front of the bot faces the scoring hub (speaker/center hub),
 * preparing for a shot. Uses the robot's current odometry pose to calculate the required heading.
 *
 * <p>The hub position is determined by alliance color automatically via DriverStation.
 */
public class RotateToHubCommand extends Command {

  // ── Hub positions on the 2025 Reefscape field (meters, WPILib origin) ──
  // Adjust these to match your actual field measurements / game manual.
  private static final Translation2d BLUE_HUB = new Translation2d(0.0, 5.547868); // Blue speaker opening
  private static final Translation2d RED_HUB  = new Translation2d(16.541748, 5.547868); // Red speaker opening

  // ── Tuning ──────────────────────────────────────────────────────────────
  private static final double kP             = 0.075; // Proportional gain  (tune on robot)
  private static final double kI             = 0.0;   // Integral gain
  private static final double kD             = 0.004; // Derivative gain
  private static final double TOLERANCE_DEG  = 1.5;   // Degrees — "close enough"
  private static final double MAX_TURN_SPEED = 0.5;   // Max chassis omega (-1 to 1 scale), tune as needed

  // ── State ────────────────────────────────────────────────────────────────
  private final SwerveDrive  swerveDrive;
  private final Vision       vision;
  private final PIDController pidController;
  private       Translation2d targetHub;

  /**
   * Creates a new RotateToHubCommand.
   *
   * @param swerveDrive The swerve drive subsystem used for movement and pose.
   * @param vision      The vision subsystem used to keep pose estimation fresh.
   */
  public RotateToHubCommand(SwerveDrive swerveDrive, Vision vision) {
    this.swerveDrive = swerveDrive;
    this.vision      = vision;

    pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-180.0, 180.0); // Handle the ±180° wrap-around
    pidController.setTolerance(TOLERANCE_DEG);

    // NOTE: Add requirements for whichever subsystem wraps swerveDrive in your project,
    // e.g.  addRequirements(driveSubsystem);
  }

  // ── Command lifecycle ────────────────────────────────────────────────────

  @Override
  public void initialize() {
    // Determine hub position by alliance at command start
    boolean isBlue = DriverStation.getAlliance()
                                  .map(a -> a == Alliance.Blue)
                                  .orElse(true); // default blue if unknown
    targetHub = isBlue ? BLUE_HUB : RED_HUB;

    pidController.reset();
  }

  @Override
  public void execute() {
    // Keep vision pose estimation current
    vision.updatePoseEstimation(swerveDrive);

    Pose2d currentPose = swerveDrive.getPose();
    double targetAngleDeg = calculateAngleToHub(currentPose.getTranslation(), targetHub);

    double currentAngleDeg = currentPose.getRotation().getDegrees();
    double rotationOutput   = pidController.calculate(currentAngleDeg, targetAngleDeg);

    // Clamp so we don't spin too fast
    rotationOutput = MathUtil.clamp(rotationOutput, -MAX_TURN_SPEED, MAX_TURN_SPEED);

    // Drive with zero translation, only rotation
    swerveDrive.drive(
        new Translation2d(0, 0), // no translation
        rotationOutput,          // omega (rad/s or %-ish depending on YAGSL config)
        true,                    // field-relative
        false                    // open loop
    );
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motion when done or interrupted
    swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
  }

  // ── Helpers ──────────────────────────────────────────────────────────────

  /**
   * Calculates the field-relative angle (degrees) the robot needs to face so its
   * front points at the hub.
   *
   * @param robotTranslation Current robot XY position on the field.
   * @param hub              Target hub XY position on the field.
   * @return Target heading in degrees, WPILib convention (CCW positive, 0 = field +X).
   */
  private double calculateAngleToHub(Translation2d robotTranslation, Translation2d hub) {
    Translation2d delta = hub.minus(robotTranslation);
    return new Rotation2d(delta.getX(), delta.getY()).getDegrees();
  }
}//woop woop your mom said so 