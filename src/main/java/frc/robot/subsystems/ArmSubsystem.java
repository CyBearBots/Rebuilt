package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    private static final double ARM_UP_LIMIT   = ArmConstants.armUpLimit;
    private static final double ARM_DOWN_LIMIT = ArmConstants.armDownLimit;

    private double kPUp   = ArmConstants.kPUp;
    private double kPDown = ArmConstants.kPDown;
    private double kI     = ArmConstants.kI;
    private double kD     = ArmConstants.kD;

    public ArmSubsystem() {
        motor = new SparkMax(12, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            .p(kPUp, ClosedLoopSlot.kSlot0)
            .i(kI,   ClosedLoopSlot.kSlot0)
            .d(kD,   ClosedLoopSlot.kSlot0)

            .p(kPDown, ClosedLoopSlot.kSlot1)
            .i(kI,     ClosedLoopSlot.kSlot1)
            .d(kD,     ClosedLoopSlot.kSlot1)

            .outputRange(-1.0, 1.0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pid = motor.getClosedLoopController();

        SmartDashboard.putNumber("Arm kP Up", kPUp);
        SmartDashboard.putNumber("Arm kP Down", kPDown);
        SmartDashboard.putNumber("Arm kI", kI);
        SmartDashboard.putNumber("Arm kD", kD);
    }

    public void setPosition(double target, ClosedLoopSlot slot){
        pid.setSetpoint(
            target,
            SparkBase.ControlType.kPosition,
            slot,
            0
        );
    }

    public void armUp() {
        setPosition(ARM_UP_LIMIT, ClosedLoopSlot.kSlot1); 
    }

    public void armDown() {
        setPosition(ARM_DOWN_LIMIT, ClosedLoopSlot.kSlot0); 
    }

    public void stop() {
        pid.setSetpoint(
            0,
            SparkBase.ControlType.kDutyCycle,
            ClosedLoopSlot.kSlot0,
            0
        );
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean atPosition(double target){
        return Math.abs(getPosition() - target) < 0.5;
    }

    @Override
    public void periodic() {

        double newPUp   = SmartDashboard.getNumber("Arm kP Up", kPUp);
        double newPDown = SmartDashboard.getNumber("Arm kP Down", kPDown);
        double newI     = SmartDashboard.getNumber("Arm kI", kI);
        double newD     = SmartDashboard.getNumber("Arm kD", kD);

        if (newPUp != kPUp || newPDown != kPDown || newI != kI || newD != kD) {

            kPUp = newPUp;
            kPDown = newPDown;
            kI = newI;
            kD = newD;

            SparkMaxConfig config = new SparkMaxConfig();

            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

                .p(kPUp, ClosedLoopSlot.kSlot0)
                .i(kI,   ClosedLoopSlot.kSlot0)
                .d(kD,   ClosedLoopSlot.kSlot0)

                .p(kPDown, ClosedLoopSlot.kSlot1)
                .i(kI,     ClosedLoopSlot.kSlot1)
                .d(kD,     ClosedLoopSlot.kSlot1)

                .outputRange(-1.0, 1.0);

            motor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );
        }

        SmartDashboard.putNumber("Arm Position", getPosition());
    }
}