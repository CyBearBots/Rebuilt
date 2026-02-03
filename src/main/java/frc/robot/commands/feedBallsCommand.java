package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;


//Feeds balls rotating around the hopper to the shooter
public class feedBallsCommand extends Command {
    
    private final HopperSubsystem hopper; 

    public feedBallsCommand(HopperSubsystem hopper) {  // Changed parameter type
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
    
