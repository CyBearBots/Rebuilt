package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

//Controls the position of the intake arm
//made by wongtonsoup
public class armDownCommand extends Command {
    private final ArmSubsystem arm;

    public armDownCommand(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        arm.armDown();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}