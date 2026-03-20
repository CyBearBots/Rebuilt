package frc.robot.commands;

public class thiefShootCommand {
    package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * THIEF MODE - Maximum power shooting for launching balls from the enemy zone back to our side.
 * Both motors run at full blast.
 */
public class thiefShootCommand extends Command {

    private final ShooterSubsystem shooter;

    // Crank it — full send
    private static final double THIEF_TOP_RPM    = -5400; // maxing out a NEO is ~5700 RPM
    private static final double THIEF_BOTTOM_RPM = -5400 * 1.0; // equal spin for max distance

    public thiefCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(THIEF_TOP_RPM, THIEF_BOTTOM_RPM);
    }

    @Override
    public void execute() {
        shooter.setTargetRPM(THIEF_TOP_RPM, THIEF_BOTTOM_RPM);
    }

    @Override
    public boolean isFinished() {
        return false; // hold until button released
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
    
}
