package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private SparkMax hopperMotor;
    private SparkMax feederMotor;

    private final Timer pulseTimer = new Timer();

    private static final double RUN_SPEED = -0.4;
    private static final double SLOW_SPEED = -0.1;
    private static final double REVERSE_SPEED = 0.4;

    private static final double FAST_TIME = 0.3;
    private static final double SLOW_TIME = 0.2;

    private boolean isSpinning = false;
    private boolean isReversing = false;

    public HopperSubsystem() {
        hopperMotor = new SparkMax(15, MotorType.kBrushless);
        feederMotor = new SparkMax(11, MotorType.kBrushless);
    }

    public void spin() {
        isSpinning = true;
        isReversing = false;

        feederMotor.set(0.7);
        pulseTimer.restart();
    }

    public void reverse() {
        isReversing = true;
        isSpinning = false;

        feederMotor.set(0); // maybe
    }

    public void stop() {
        isSpinning = false;
        isReversing = false;

        hopperMotor.set(0);
        feederMotor.set(0);
        pulseTimer.stop();
    }

    @Override
    public void periodic() {
        double feederCurrent = feederMotor.getOutputCurrent();

        SmartDashboard.putNumber("Primer Current: ", feederCurrent);

        if(feederCurrent > 35){
            reverse();
        }

        if (isReversing) {
            hopperMotor.set(REVERSE_SPEED);
            return;
        }

        if (!isSpinning) {
            hopperMotor.set(0);
            return;
        }

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