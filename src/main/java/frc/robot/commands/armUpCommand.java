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
        arm.armDown();
    }

    @Override
    public boolean isFinished(){
        return arm.atPosition(ArmConstants.armDownLimit);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}


















/*
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class armUpCommand extends Command {
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public armUpCommand(ArmSubsystem arm, IntakeSubsystem intake) {
        this.arm = arm;
        this.intake = intake;
        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        arm.armUp();
        intake.spin(); // or slowSpin() if you want it gentler
    }

    @Override
    public boolean isFinished() {
        return arm.atPosition(ArmConstants.armUpLimit);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        intake.brake();
    }
}
    */