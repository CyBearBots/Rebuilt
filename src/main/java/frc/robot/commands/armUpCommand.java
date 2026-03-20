package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

//Controls the position of the intake arm
//made by wongtonsoup
public class armUpCommand extends Command {
    private final ArmSubsystem arm;

    public armUpCommand(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        arm.armUp();
    }

    @Override
    public boolean isFinished(){
        return arm.atPosition(ArmConstants.armUpLimit);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}