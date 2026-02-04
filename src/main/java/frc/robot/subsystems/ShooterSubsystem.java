package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax motor13;
    private final SparkMax motor9;

    public ShooterSubsystem() {
        motor13 = new SparkMax(13, MotorType.kBrushless);
        motor9  = new SparkMax(9,  MotorType.kBrushless);

        configureMotor(motor13);
        configureMotor(motor9);
    }

    private void configureMotor(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(40);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /** Spin shooter motors */
    public void spin() {
        motor13.set(1.00);
        motor9.set(-0.4);
    }

    /** Stop shooter motors */
    public void stop() {
        motor13.stopMotor();
        motor9.stopMotor();
    }
}
