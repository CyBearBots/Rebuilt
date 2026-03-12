/*package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkClosedLoopController leftPID;
    private final SparkClosedLoopController rightPID;

    // TODO: tune these
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // TODO: find these by testing on the robot
    public static final double UP_POSITION = 100.0;
    public static final double DOWN_POSITION = 0.0;

    public ClimberSubsystem() {
        leftMotor  = new SparkMax(1, MotorType.kBrushless); // TODO: set CAN ID
        rightMotor = new SparkMax(2, MotorType.kBrushless); // TODO: set CAN ID

        leftPID  = leftMotor.getClosedLoopController();
        rightPID = rightMotor.getClosedLoopController();

        configureMotor(leftMotor, false);
        configureMotor(rightMotor, true); // inverted since they face opposite directions
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(40)
              .inverted(inverted);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-1.0, 1.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void goToPosition(double position) {
        leftPID.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
        rightPID.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    }

    public void stop() {
        leftPID.setSetpoint(getLeftPosition(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
        rightPID.setSetpoint(getRightPosition(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    }

    public double getLeftPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return rightMotor.getEncoder().getPosition();
    }

    public boolean atPosition(double target, double tolerance) {
        return Math.abs(getLeftPosition() - target) < tolerance
            && Math.abs(getRightPosition() - target) < tolerance;
    }

    public void resetEncoders() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
    }
}

/*
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private static final double CLIMB_SPEED = 0.5; // TODO: tune

    public ClimberSubsystem() {
        leftMotor  = new SparkMax(1, MotorType.kBrushless); // TODO: CAN ID
        rightMotor = new SparkMax(2, MotorType.kBrushless); // TODO: CAN ID
    }

    public void climbUp() {
        leftMotor.set(CLIMB_SPEED);
        rightMotor.set(-CLIMB_SPEED); // opposite direction
    }

    public void climbDown() {
        leftMotor.set(-CLIMB_SPEED);
        rightMotor.set(CLIMB_SPEED); // opposite direction
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
    */