package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class ReverseHopperCommand extends Command {

    private final HopperSubsystem hopper;

    public ReverseHopperCommand(HopperSubsystem hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        hopper.reverse();
    }

    @Override
    public void execute() {
        hopper.reverse(); 
    }

    @Override
    public boolean isFinished() {
        return false; // run until the button is released (whileTrue handles that)
    }

    @Override
    public void end(boolean interrupted) {
        hopper.brake();
    }
}