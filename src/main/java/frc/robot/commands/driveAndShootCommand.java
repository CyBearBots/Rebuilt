package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveDrive;

import java.util.Optional;

public class driveAndShootCommand extends Command {

    private static final double MAX_DRIVE = 0.4;
    private static final double MAX_TURN  = 0.4;

    private static final double STOP_DISTANCE = 1.2;

    private static final double kP_TRANSLATE = 2.0;
    private static final double kP_ROTATE = 0.08;
    private static final double kD_ROTATE = 0.004;

    private final SwerveDrive swerveDrive;
    private final Vision vision;
    private final ShooterSubsystem shooter;

    private final PIDController xPID = new PIDController(kP_TRANSLATE, 0, 0);
    private final PIDController yPID = new PIDController(kP_TRANSLATE, 0, 0);
    private final PIDController rotPID = new PIDController(kP_ROTATE, 0, kD_ROTATE);

    private int tagId1;
    private int tagId2;

    private static final int BLUE_TAG_1 = 25;
    private static final int BLUE_TAG_2 = 26;
    private static final int RED_TAG_1  = 9;
    private static final int RED_TAG_2  = 10;

    private double topRPM;
    private boolean aligned = false;

    public driveAndShootCommand(SwerveDrive swerveDrive, Vision vision, ShooterSubsystem shooter) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.shooter = shooter;

        rotPID.enableContinuousInput(-180, 180);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

        boolean isBlue = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Blue)
            .orElse(true);

        tagId1 = isBlue ? BLUE_TAG_1 : RED_TAG_1;
        tagId2 = isBlue ? BLUE_TAG_2 : RED_TAG_2;

        aligned = false;

        xPID.reset();
        yPID.reset();
        rotPID.reset();
    }

    private Translation2d getHubCenter() {

        Optional<Pose3d> t1 = Vision.fieldLayout.getTagPose(tagId1);
        Optional<Pose3d> t2 = Vision.fieldLayout.getTagPose(tagId2);

        if (t1.isPresent() && t2.isPresent()) {
            return new Translation2d(
                (t1.get().getX() + t2.get().getX()) / 2.0,
                (t1.get().getY() + t2.get().getY()) / 2.0
            );
        }

        if (t1.isPresent()) return t1.get().toPose2d().getTranslation();
        if (t2.isPresent()) return t2.get().toPose2d().getTranslation();

        return null;
    }

    @Override
    public void execute() {

        vision.updatePoseEstimation(swerveDrive);

        Translation2d hub = getHubCenter();

        if (hub == null) {
            swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
            return;
        }

        Pose2d current = swerveDrive.getPose();

        // direction to hub
        Translation2d toHub = hub.minus(current.getTranslation());
        double angleToHub = Math.atan2(toHub.getY(), toHub.getX());

        // stop point in front of hub
        Translation2d stopPoint = hub.minus(
            new Translation2d(
                Math.cos(angleToHub) * STOP_DISTANCE,
                Math.sin(angleToHub) * STOP_DISTANCE
            )
        );

        double xOut = MathUtil.clamp(
            xPID.calculate(current.getX(), stopPoint.getX()),
            -MAX_DRIVE, MAX_DRIVE
        );

        double yOut = MathUtil.clamp(
            yPID.calculate(current.getY(), stopPoint.getY()),
            -MAX_DRIVE, MAX_DRIVE
        );

        double rotOut = MathUtil.clamp(
            rotPID.calculate(current.getRotation().getDegrees(), Math.toDegrees(angleToHub)),
            -MAX_TURN, MAX_TURN
        );

        swerveDrive.drive(new Translation2d(xOut, yOut), rotOut, true, false);

        // check alignment
        if (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
            aligned = true;
        }

        if (aligned) {

            double distance = shooter.getDistanceToHub(vision, tagId1, tagId2);
            topRPM = shooter.calculateRPMFromDistance(distance);

            shooter.setTargetRPM(
                topRPM,
                topRPM * ShooterConstants.spinRatio
            );
        }
    }

    @Override
    public boolean isFinished() {
        return aligned &&
               shooter.atSetpoint(
                   topRPM,
                   topRPM * ShooterConstants.spinRatio,
                   ShooterConstants.rpmTolerance
               );
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new Translation2d(0, 0), 0, true, false);
        shooter.stop();
    }
}