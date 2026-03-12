/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpCommand extends Command {

    private final ClimberSubsystem climber;
    private static final double TOLERANCE = 2.0; // TODO: tune

    public ClimbUpCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.goToPosition(ClimberSubsystem.UP_POSITION);
    }

    @Override
    public boolean isFinished() {
        return climber.atPosition(ClimberSubsystem.UP_POSITION, TOLERANCE);
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

public class ClimbUpCommand extends Command {

    private final ClimberSubsystem climber;

    public ClimbUpCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.climbUp();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
    */