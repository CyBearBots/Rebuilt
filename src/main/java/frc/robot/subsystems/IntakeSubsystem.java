package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;


public class IntakeSubsystem extends SubsystemBase {

    private SparkMax motor;

    public IntakeSubsystem() {
        motor = new SparkMax(13, MotorType.kBrushless);
    }

    public void spin() {
        motor.set(-0.6);
    }

    public void slowSpin() {
    motor.set(-0.2); 
    }

    public void brake() {
        motor.set(0.0);
    }
    
    @Override
    public void periodic() {}
}
