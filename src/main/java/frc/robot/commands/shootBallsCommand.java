package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class shootBallsCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double topRPM;
    private final double bottomRPM;

    public shootBallsCommand(ShooterSubsystem shooter, double topRPM, double bottomRPM) {
        this.shooter = shooter;
        this.topRPM = topRPM;
        this.bottomRPM = bottomRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(topRPM, bottomRPM);
    }

    @Override
    public void execute() {
        shooter.setTargetRPM(topRPM, bottomRPM);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}