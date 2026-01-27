package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;


public class ShooterSubsystem extends SubsystemBase {
    
    private SparkMax flyWheelMotor;

    double highVelocity = ShooterConstants.highVelocity;
    double lowVelocity = ShooterConstants.lowVelocity;

    double stepSizes[] = ShooterConstants.stepSizes;

    int stepIndex = 1;

    // two wheels, spinning vertically 

    public ShooterSubsystem(){
        flyWheelMotor = new SparkMax(ShooterConstants.flywheelMotorId, MotorType.kBrushless);

        PIDController pidController = new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V, ShooterConstants.A);
    }

    @Override
    public void periodic() {}

}
