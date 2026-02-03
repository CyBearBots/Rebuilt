package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

//Rotates the hopper motor to feed balls into the shooter
public class hopperFeedCommand extends Command {
    
    private final HopperSubsystem hopper;

    public hopperFeedCommand(HopperSubsystem hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        hopper.spin();
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stop();
    }
}
