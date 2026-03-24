package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

//Shoots balls out of the robot (top motor + middle motor)
public class passBallsCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double topRPM;
    private final double bottomRPM;

    double distanceToHub;

    public passBallsCommand(ShooterSubsystem shooter, double topRPM, double bottomRPM) {
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
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return shooter.atSetpoint(
    //         topRPM,
    //         bottomRPM,
    //         ShooterConstants.rpmTolerance
    //     );
    // }

    @Override
    public boolean isFinished() {
        return false;
    }
}