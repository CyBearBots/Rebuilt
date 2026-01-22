package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    
    private SparkMax motor;

    public ArmSubsystem(int motorId){
        motor = new SparkMax(motorId, MotorType.kBrushless); 
    }
}
