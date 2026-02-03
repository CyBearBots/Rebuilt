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

public class HopperSubsystem extends SubsystemBase {
    
    private SparkMax hopperMotor;
    private SparkMax feederMotor;

    public HopperSubsystem() {
        hopperMotor = new SparkMax(11, MotorType.kBrushless);
        feederMotor = new SparkMax(14, MotorType.kBrushed);
    }

    public void spin() {
        hopperMotor.set(-0.8);
        feederMotor.set(0.7);
    }

    public void stop() {
        hopperMotor.stopMotor();
        feederMotor.stopMotor();
    }

    @Override
    public void periodic() {}
}
