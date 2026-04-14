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

    private double spinRatioNew;


    private static final double[] DISTANCES = {
        0.0,  // very close
        1.0,  // point 1
        2.0,  // point 2
        3.0,  // point 3
        4.5   // far shot
    };

    private static final double[] RPM_VALUES = {
        2000,
        2600,
        3200,
        3900,
        4600
    };

    public calculateRPMSCommand(ShooterSubsystem shooterSubsystem, Vision vision) {
        this.shooterSubsystem = shooterSubsystem;
        this.vision = vision;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        spinRatioNew = ShooterConstants.spinRatio;
    }

    // 🔥 NEW: interpolation-based RPM calculation
    private double interpolateRPM(double distance) {

        if (distance <= DISTANCES[0]) {
            return RPM_VALUES[0];
        }

        if (distance >= DISTANCES[DISTANCES.length - 1]) {
            return RPM_VALUES[RPM_VALUES.length - 1];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {

            double d1 = DISTANCES[i];
            double d2 = DISTANCES[i + 1];

            if (distance >= d1 && distance <= d2) {

                double r1 = RPM_VALUES[i];
                double r2 = RPM_VALUES[i + 1];

                double t = (distance - d1) / (d2 - d1);

                return r1 + t * (r2 - r1);
            }
        }

        return ShooterConstants.topRPMDefault;
    }

    @Override
    public void execute() {

        double distance =
            shooterSubsystem.getDistanceToHub(vision, HUB_TAG_ID_1, HUB_TAG_ID_2);

        double topRPM;

        if (distance <= 0) {
            topRPM = ShooterConstants.topRPMDefault;
            SmartDashboard.putString("VisionShot/Status", "No tag — using default");
        } else {
            topRPM = interpolateRPM(distance);
            SmartDashboard.putString("VisionShot/Status", "Tag locked");
        }

        // 🔧 spin ratio adjustment
        if (distance < 1.524) { // 5 ft
            spinRatioNew = ShooterConstants.spinRationUnder5Ft;
        } else {
            spinRatioNew = ShooterConstants.spinRatio;
        }

        double bottomRPM = topRPM * spinRatioNew;

        SmartDashboard.putNumber("VisionShot/Distance", distance);
        SmartDashboard.putNumber("VisionShot/TopRPM", topRPM);
        SmartDashboard.putNumber("VisionShot/BottomRPM", bottomRPM);
        SmartDashboard.putNumber("SpinRatioNew", spinRatioNew);

        shooterSubsystem.setTargetRPM(topRPM, bottomRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
/*package frc.robot.commands;

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
}*/
    