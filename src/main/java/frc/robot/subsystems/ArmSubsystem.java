package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

    private SparkMax motor;

    public ArmSubsystem() {
        motor = new SparkMax(12, MotorType.kBrushless);
    }

    private final double ARM_UP_SPEED = 0.5; 
    private final double ARM_DOWN_SPEED = -0.5;

    public void armUp() {
        motor.set(ARM_UP_SPEED);
    }

    public void armDown() {
        motor.set(ARM_DOWN_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void periodic() {}



}
