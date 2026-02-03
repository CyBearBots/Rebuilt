package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//Shoots balls out of the robot (top motor + middle motor)
public class shootBallsCommand extends Command { 
    
    private final ShooterSubsystem shooter;

    public shootBallsCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.spin();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}