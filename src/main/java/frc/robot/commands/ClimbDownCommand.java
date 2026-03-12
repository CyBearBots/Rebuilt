/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {

    private final ClimberSubsystem climber;
    private static final double TOLERANCE = 2.0; // TODO: tune

    public ClimbDownCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.goToPosition(ClimberSubsystem.DOWN_POSITION);
    }

    @Override
    public boolean isFinished() {
        return climber.atPosition(ClimberSubsystem.DOWN_POSITION, TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}

/* 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {

    private final ClimberSubsystem climber;

    public ClimbDownCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.climbDown();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
    */