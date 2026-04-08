package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private SparkMax hopperMotor;
    private SparkMax feederMotor;

    private final Timer pulseTimer    = new Timer();
    private final Timer reverseTimer  = new Timer();
    private final Timer cooldownTimer = new Timer();

    private static final double RUN_SPEED     = 0.7;
    private static final double SLOW_SPEED    = 0.3;
    private static final double REVERSE_SPEED = -0.4;
    private static final double REVERSE_TIME  =  -2.0;
    private static final double COOLDOWN_TIME =  1.0;
    private static final double FAST_TIME     =  -0.3;
    private static final double SLOW_TIME     =  -0.2;

    private boolean isSpinning  = false;
    private boolean isReversing = false;

    public HopperSubsystem() {
        hopperMotor = new SparkMax(15, MotorType.kBrushless);
        feederMotor = new SparkMax(11, MotorType.kBrushless);
        cooldownTimer.start();
    }

    public void spin() {
        isSpinning  = true;
        isReversing = false;
        feederMotor.set(1.0);
        pulseTimer.restart();
        reverseTimer.stop();
    }

    public void reverse() {
        if (!isReversing) {
            isReversing = true;
            isSpinning  = false;
            reverseTimer.restart();
            pulseTimer.stop();
        }
        hopperMotor.set(REVERSE_SPEED);
        feederMotor.set(-0.5);// reverse feeder too
    }

    public void stop() {
        isSpinning  = false;
        isReversing = false;
        hopperMotor.set(0);
        feederMotor.set(0);
        pulseTimer.stop();
        reverseTimer.stop();
    }

    @Override
    public void periodic() {
        double feederCurrent = feederMotor.getOutputCurrent();
        SmartDashboard.putNumber("Primer Current: ", feederCurrent);

        // Auto-reverse on current spike (jam detection)
        // TODO: measure current limit before enabling — keeps auto-reversing without proper threshold
        // if (feederCurrent > 26 && !isReversing && cooldownTimer.get() > COOLDOWN_TIME) {
        //     isReversing = true;
        //     isSpinning  = false;
        //     reverseTimer.restart();
        //     pulseTimer.stop();
        // }

        // Reversing phase
        if (isReversing) {
            hopperMotor.set(REVERSE_SPEED);
            feederMotor.set(-0.5);

            if (reverseTimer.get() > REVERSE_TIME) {
                isReversing = false;
                isSpinning  = true;
                cooldownTimer.restart();
                pulseTimer.restart();
                feederMotor.set(0.7);
            }
            return;
        }

        //Not spinning
        if (!isSpinning) {
            hopperMotor.set(0);
            feederMotor.set(0);
            return;
        }

        //Pulse pattern 
        double t               = pulseTimer.get();
        double cycleTime       = FAST_TIME + SLOW_TIME;
        double positionInCycle = t % cycleTime;

        feederMotor.set(0.7);

        if (positionInCycle < FAST_TIME) {
            hopperMotor.set(RUN_SPEED);   // fast pulse
        } else {
            hopperMotor.set(SLOW_SPEED);  // slow pulse
        }
    }
}