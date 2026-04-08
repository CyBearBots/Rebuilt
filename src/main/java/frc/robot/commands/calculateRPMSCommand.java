package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class calculateRPMSCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final Vision vision;

    private static final int HUB_TAG_ID_1 = 9;
    private static final int HUB_TAG_ID_2 = 10;

    double spinRatioNew;

    public calculateRPMSCommand(ShooterSubsystem shooterSubsystem, Vision vision) {
        this.shooterSubsystem = shooterSubsystem;
        this.vision = vision;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        spinRatioNew = ShooterConstants.spinRatio;
    }

    @Override
    public void execute() {
        double distance = shooterSubsystem.getDistanceToHub(vision, HUB_TAG_ID_1, HUB_TAG_ID_2);

        double topRPM;

        if (distance <= 0) {
            // No tag visible  fall back to default RPM so the shooter still spins up
            topRPM = ShooterConstants.topRPMDefault;
            SmartDashboard.putString("VisionShot/Status", "No tag — using default");
        } else {
            topRPM = shooterSubsystem.calculateRPMFromDistance(distance);
            SmartDashboard.putString("VisionShot/Status", "Tag locked");
        }

        if(distance < 1.524){ // 5 feet
            spinRatioNew = ShooterConstants.spinRationUnder5Ft;
        }else{
            spinRatioNew = ShooterConstants.spinRatio;
        }

        double bottomRPM = topRPM * spinRatioNew;

        SmartDashboard.putNumber("VisionShot/Distance",   distance);
        SmartDashboard.putNumber("VisionShot/TopRPM",     topRPM);
        SmartDashboard.putNumber("VisionShot/BottomRPM",  bottomRPM);
        SmartDashboard.putNumber("SpinRatioNew", spinRatioNew);

        shooterSubsystem.setTargetRPM(topRPM, bottomRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button
    }
}