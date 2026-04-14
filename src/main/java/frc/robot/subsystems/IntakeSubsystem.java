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
/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class IntakeSubsystem extends SubsystemBase {

    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    public IntakeSubsystem() {
        motor = new SparkMax(13, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.kP)
            .i(IntakeConstants.kI)
            .d(IntakeConstants.kD)
            .velocityFF(IntakeConstants.kV / 6784) // kV / max RPM
            .outputRange(-1.0, 1.0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        pid = motor.getClosedLoopController();
    }

    private void setRPM(double rpm) {
        pid.setReference(rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void spin() {
        setRPM(IntakeConstants.spinRPM);
    }

    public void slowSpin() {
        setRPM(IntakeConstants.slowSpinRPM);
    }

    public void brake() {
        motor.set(0.0);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake RPM", getRPM());
    }
}
    */