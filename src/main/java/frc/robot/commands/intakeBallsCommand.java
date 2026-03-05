package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

//Intakes balls from the field into the robot
public class intakeBallsCommand extends Command {
    private final IntakeSubsystem intake;

    public intakeBallsCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.spin();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
