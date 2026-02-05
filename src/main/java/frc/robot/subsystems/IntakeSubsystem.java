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
        motor = new SparkMax(12, MotorType.kBrushless);
    }

    public void spin() {
        motor.set(0.7);
    }

    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void periodic() {}
}
