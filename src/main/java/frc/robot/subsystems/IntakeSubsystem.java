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

    public IntakeSubsystem(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    //values of intake and eject should be opposite (signs)

    public void stopMotor(){
        motor.set(0);
        // can substitute with motor.stopMotor();
    }
    
    @Override
    public void periodic() {}
}
