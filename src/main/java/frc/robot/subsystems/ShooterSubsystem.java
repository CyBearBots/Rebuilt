package frc.robot.subsystems;

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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax motor10;
    private final SparkMax motor9;

    private final SparkClosedLoopController pid13;
    private final SparkClosedLoopController pid9;

    private final SimpleMotorFeedforward feedforward;

    private double targetRPM = 0.0;

    public ShooterSubsystem() {
        motor10 = new SparkMax(10, MotorType.kBrushless);
        motor9  = new SparkMax(9,  MotorType.kBrushless);

        pid13 = motor10.getClosedLoopController();
        pid9 = motor9.getClosedLoopController();

        feedforward = new SimpleMotorFeedforward(
            ShooterConstants.kS,
            ShooterConstants.kV,
            ShooterConstants.kA
        );

        configureMotor(motor10);
        configureMotor(motor9);
    }

    private void configureMotor(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(40);

         config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kP)
            .i(ShooterConstants.kI)
            .d(ShooterConstants.kD)
            .outputRange(-1.0, 1.0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setTargetRPM(double topRPM, double bottomRPM) {

        targetRPM = topRPM;

        double ffVoltsTop = feedforward.calculate(topRPM / 60);
        double ffVoltsBottom = feedforward.calculate(bottomRPM / 60);

        SmartDashboard.putNumber("VoltsTop", ffVoltsTop);
        SmartDashboard.putNumber("VoltsBottom", ffVoltsBottom);

        // TOP MOTOR
        pid9.setSetpoint(
            topRPM,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVoltsTop
        );

        // BOTTOM MOTOR
        pid13.setSetpoint(
            bottomRPM,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVoltsBottom
        );
    }

    // /** Spin shooter motors */
    // public void spin() {
    //     motor13.set(-0.23);//bottom, .20
    //     motor9.set(-0.30);//top, .25
    // }

    /** Stop shooter motors */
    public void stop() {
        targetRPM = 0.0;
        motor10.stopMotor();
        motor9.stopMotor();
    }

    public double getBottomRPM(){
        return motor10.getEncoder().getVelocity();
    }

    public double getTopRPM(){
        return motor9.getEncoder().getVelocity();
    }

    public boolean atSetpoint(double topTarget, double bottomTarget, double toleranceRPM) {
        return Math.abs(getTopRPM() - topTarget) < toleranceRPM
            && Math.abs(getBottomRPM() - bottomTarget) < toleranceRPM;
    }
}


