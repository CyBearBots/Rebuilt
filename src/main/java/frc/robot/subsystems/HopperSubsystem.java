package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
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

    private boolean isSpinning = false;
    private final Timer pulseTimer = new Timer();

    private static final double RUN_SPEED = -0.4;
    private static final double SLOW_SPEED = -0.1;  //Ask Cole

    private static final double FAST_TIME = 0.3;
    private static final double SLOW_TIME = 0.2;

    public HopperSubsystem() {
        hopperMotor = new SparkMax(15, MotorType.kBrushless);
        feederMotor = new SparkMax(11, MotorType.kBrushless);
    }

    public void spin() {
        isSpinning = true;
        feederMotor.set(0.7);
        pulseTimer.restart();
    }

    public void stop() {
        isSpinning = false;
        hopperMotor.set(0);
        feederMotor.set(0);
        pulseTimer.stop();
    }

    @Override
    public void periodic() {
        if (!isSpinning) return;

        double t = pulseTimer.get();
        double cycleTime = FAST_TIME + SLOW_TIME;
        double positionInCycle = t % cycleTime;

        if (positionInCycle < FAST_TIME) {
            hopperMotor.set(RUN_SPEED);
        } else {
            hopperMotor.set(SLOW_SPEED);
        }
    }
}
