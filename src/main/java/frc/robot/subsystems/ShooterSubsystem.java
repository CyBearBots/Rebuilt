package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax flywheelMotor;
    private final SparkClosedLoopController pidController;

    private final SimpleMotorFeedforward feedforward;

    private double targetRPM = 0.0;

    public ShooterSubsystem() {
        flywheelMotor = new SparkMax(
            ShooterConstants.flywheelMotorId,
            MotorType.kBrushless
        );

        pidController = flywheelMotor.getClosedLoopController();

        feedforward = new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA
        );

        configureMotor();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(40);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kP)
            .i(ShooterConstants.kI)
            .d(ShooterConstants.kD)
            .outputRange(-1.0, 1.0);

        flywheelMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;

        double ffVolts = feedforward.calculate(rpm);

        pidController.setSetpoint(
            rpm,                                   // Velocity setpoint (RPM)
            SparkBase.ControlType.kVelocity,       // Velocity control
            ClosedLoopSlot.kSlot0,                 // PID slot
            ffVolts                                // Arbitrary feedforward (V)
        );
    }

    public void stop() {
        targetRPM = 0.0;
        flywheelMotor.stopMotor();
    }

    public double getRPM() {
        return flywheelMotor.getEncoder().getVelocity();
    }

    public boolean atSetpoint(double toleranceRPM) {
        return Math.abs(getRPM() - targetRPM) < toleranceRPM;
    }
}
