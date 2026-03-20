package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

public class ArmSubsystem extends SubsystemBase {

    private SparkMax motor;
    private RelativeEncoder encoder;

    private static final double ARM_UP_LIMIT   = -5.00;
    private static final double ARM_DOWN_LIMIT = -20.00;

    private final double ARM_UP_SPEED   =  0.25;
    private final double ARM_DOWN_SPEED = -0.3;

    public ArmSubsystem() {
        motor   = new SparkMax(12, MotorType.kBrushless);


        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, SparkMax.ResetMode.kNoResetSafeParameters,
                                SparkMax.PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder(); 
        encoder.setPosition(0);
    }

    public void armUp() {
        if (encoder.getPosition() < ARM_UP_LIMIT) {
            motor.set(ARM_UP_SPEED);
        } else {
            stop();
        }
        //motor.set(ARM_UP_SPEED);
    }

    public void armDown() {
        if (encoder.getPosition() > ARM_DOWN_LIMIT) {
            motor.set(ARM_DOWN_SPEED);
        } else {
            stop();
        }
       // motor.set(ARM_DOWN_SPEED);
    }

    public void stop() {
        motor.stopMotor(); // 
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());    
    }
}

