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

public class ArmSubsystem extends SubsystemBase {

    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    private static final double ARM_UP_LIMIT   = ArmConstants.armUpLimit;
    private static final double ARM_DOWN_LIMIT = ArmConstants.armDownLimit;

    // private final double ARM_UP_SPEED   =  0.25;
    // private final double ARM_DOWN_SPEED = -0.3;

    private double kP = ArmConstants.kP;
    private double kI = ArmConstants.kI;
    private double kD = ArmConstants.kD;

    public ArmSubsystem() {
        motor = new SparkMax(12, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(-1.0, 1.0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pid = motor.getClosedLoopController();

        SmartDashboard.putNumber("Arm kP", kP);
        SmartDashboard.putNumber("Arm kI", kI);
        SmartDashboard.putNumber("Arm kD", kD);
    
    }

    public void setPosition(double target){
        pid.setSetpoint(
            target, 
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            0 // no feed forward as of now
        );
    }

    public void armUp() {
        // if (encoder.getPosition() < ARM_UP_LIMIT) {
        //     motor.set(ARM_UP_SPEED);
        // } else {
        //     stop();
        // }
        // //motor.set(ARM_UP_SPEED);

        setPosition(ARM_UP_LIMIT);
    }

    public void armDown() {
    //     if (encoder.getPosition() > ARM_DOWN_LIMIT) {
    //         motor.set(ARM_DOWN_SPEED);
    //     } else {
    //         stop();
    //     }
    //    // motor.set(ARM_DOWN_SPEED);

        setPosition(ARM_DOWN_LIMIT);
    }

    public void stop() {
       // motor.stopMotor();

       // 0 since dont want motor to work when resting on bumper

       pid.setSetpoint(0, SparkBase.ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean atPosition(double target){
        return Math.abs(getPosition() - target) < 0.5;
    }


    @Override
    public void periodic() {

        // for pid tuning using the dashboard

        double newP = SmartDashboard.getNumber("Arm kP", kP);
        double newI = SmartDashboard.getNumber("Arm kI", kI);
        double newD = SmartDashboard.getNumber("Arm kD", kD);

        if (newP != kP || newI != kI || newD != kD) {
            kP = newP;
            kI = newI;
            kD = newD;

            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kP)
                .i(kI)
                .d(kD)
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
