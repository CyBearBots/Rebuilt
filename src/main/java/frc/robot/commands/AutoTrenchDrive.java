package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoTrenchDrive extends Command {

    private final SwerveSubsystem drive;

    private final PIDController xPID = new PIDController(1.8, 0, 0);
    private final PIDController yPID = new PIDController(1.8, 0, 0);
    private final PIDController rotPID = new PIDController(3.0, 0, 0);

    public AutoTrenchDrive(SwerveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);

        rotPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Pose2d getTarget() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

            return new Pose2d(13.0, 6.5, Rotation2d.fromDegrees(180));
        }

        return new Pose2d(3.5, 2.0, Rotation2d.fromDegrees(0));
    }

    @Override
    public void execute() {
        Pose2d current = drive.getPose();
        Pose2d target = getTarget();

        double x = xPID.calculate(current.getX(), target.getX());
        double y = yPID.calculate(current.getY(), target.getY());
        double rot = rotPID.calculate(
                current.getRotation().getRadians(),
                target.getRotation().getRadians()
        );

        drive.driveFieldOriented(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                        x, y, rot
                )
        );
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