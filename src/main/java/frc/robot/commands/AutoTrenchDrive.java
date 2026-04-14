package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoTrenchDrive extends Command {

    private final SwerveSubsystem drive;
    private final Vision vision;

    //private final PIDController forwardPID = new PIDController(0.05, 0, 0);
    //private final PIDController strafePID = new PIDController(0.05, 0, 0);
    private final PIDController turnPID = new PIDController(0.02, 0, 0);

    private final int[] TARGET_TAGS = {5, 6, 7, 8};

    public AutoTrenchDrive(SwerveSubsystem drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    private PhotonTrackedTarget getBestTarget() {
        PhotonTrackedTarget best = null;
        double bestScore = 999;

        for (int id : TARGET_TAGS) {

            PhotonTrackedTarget t =
                vision.getTargetFromId(id, Cameras.CAMERA_1);

            if (t != null) {
                double score = Math.abs(t.getYaw());

                if (score < bestScore) {
                    bestScore = score;
                    best = t;
                }
            }
        }

        return best;
    }

    @Override
    public void execute() {

        PhotonTrackedTarget target = getBestTarget();

        if (target == null) {
            drive.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);
            return;
        }

        double yaw = target.getYaw();

        double turn = turnPID.calculate(yaw, 0);

        double forward = (Math.abs(yaw) < 15)
            ? 0.3 * (1 - Math.abs(yaw) / 30.0)
            : 0.0;
        double strafe = 0.0;

        drive.driveFieldOriented(
            new ChassisSpeeds(forward, strafe, turn)
        );
    }

    @Override
    public void initialize() {
        //forwardPID.reset();
        //strafePID.reset();
        turnPID.reset();
    }
    @Override
    public void end(boolean interrupted) {
        drive.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);
    }   
}
/*package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class AutoTrenchDrive extends Command {

    private final SwerveSubsystem drive;
    private final Vision vision;

    private final PIDController xPID = new PIDController(1.8, 0, 0);
    private final PIDController yPID = new PIDController(1.8, 0, 0);
    private final PIDController rotPID = new PIDController(3.0, 0, 0);

    // 🔧 CHANGE THIS to whatever tag you want to track
    private final int TARGET_TAGS = 5;

    public AutoTrenchDrive(SwerveSubsystem drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);

        rotPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Pose2d getTarget() {
        try {
            // Offset = where you want robot relative to tag
            Transform2d offset = new Transform2d(
                1.0, 0.0, new Rotation2d() // 1 meter in front of tag
            );

            return Vision.getAprilTagPose(TARGET_TAG_ID, offset);

        } catch (Exception e) {
            return null;
        }
    }

    @Override
    public void execute() {
        Pose2d target = getTarget();

        if (target == null) {
            drive.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);
            return;
        }

        Pose2d current = drive.getPose();

        double x = xPID.calculate(current.getX(), target.getX());
        double y = yPID.calculate(current.getY(), target.getY());
        double rot = rotPID.calculate(
            current.getRotation().getRadians(),
            target.getRotation().getRadians()
        );

        drive.driveFieldOriented(new ChassisSpeeds(x, y, rot));
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetpoint()
            && yPID.atSetpoint()
            && rotPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);
    }
}
    */