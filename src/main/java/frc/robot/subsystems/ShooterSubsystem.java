package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

//brake motors instead
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax motor10;
    private final SparkMax motor9;

    private SparkClosedLoopController pid13;
    private SparkClosedLoopController pid9;

    private SimpleMotorFeedforward feedforward;

    private double kP = ShooterConstants.kP;
    private double kI = ShooterConstants.kI;
    private double kD = ShooterConstants.kD; 

    private double kS = ShooterConstants.kS;
    private double kV = ShooterConstants.kV;
    private double kA = ShooterConstants.kA;

    private double targetRPM = 0.0;

    PhotonCamera camera1 = new PhotonCamera("Camera1");
    PhotonCamera camera2 = new PhotonCamera("Camera2");
    double targetID = 9;
    double distance;

    public ShooterSubsystem() {
        motor10 = new SparkMax(10, MotorType.kBrushless);
        motor9  = new SparkMax(9,  MotorType.kBrushless);

        pid13 = motor10.getClosedLoopController();
        pid9 = motor9.getClosedLoopController();

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        configureMotor(motor10);
        configureMotor(motor9);

        // SmartDashboard.putNumber("Shooter kP", kP);
        // SmartDashboard.putNumber("Shooter kI", kI);
        // SmartDashboard.putNumber("Shooter kD", kD);

        // SmartDashboard.putNumber("Shooter kS", kS);
        // SmartDashboard.putNumber("Shooter kV", kV);
        // SmartDashboard.putNumber("Shooter kA", kA);

         SmartDashboard.putNumber("DistanceToHub", distance);
    }

    private void configureMotor(SparkMax motor) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(40);

         config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kP)
            .i(ShooterConstants.kI)
            .d(ShooterConstants.kD)
            .outputRange(-1.0, 1.0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }
    public double calculateRPMFromDistance(double distanceMeters) {

        // tuned, data from LSRL (4.5ft 860rpm, 8ft 1060rpm, 12ft 1160rpm)
        double kSlope = 149.97;   // how much RPM changes per meter 
        double kIntercept = 700.899; // base RPM 

        // basic linear model
        double rpm = kSlope * distanceMeters + kIntercept;

        if(distanceMeters == 0){
            return ShooterConstants.topRPMDefault;
        }

        return rpm * -1;
    }

    public void setTargetRPM(double topRPM, double bottomRPM) {

        targetRPM = topRPM;

        double ffVoltsTop = feedforward.calculate(topRPM / 60);
        double ffVoltsBottom = feedforward.calculate(bottomRPM / 60);

        SmartDashboard.putNumber("VoltsTop", ffVoltsTop);
        SmartDashboard.putNumber("VoltsBottom", ffVoltsBottom);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("AutoRPM", topRPM);

        // TOP MOTOR
        pid9.setSetpoint(
            topRPM,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVoltsTop
        );

        // BOTTOM MOTOR
        pid13.setSetpoint(
            bottomRPM,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVoltsBottom
        );
    }

    @Override
    public void periodic(){

        double newP = SmartDashboard.getNumber("Shooter kP", kP);
        double newI = SmartDashboard.getNumber("Shooter kI", kI);
        double newD = SmartDashboard.getNumber("Shooter kD", kD);

        double newKS = SmartDashboard.getNumber("Shooter kS", kS);
        double newKV = SmartDashboard.getNumber("Shooter kV", kV);
        double newKA = SmartDashboard.getNumber("Shooter kA", kA);

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(-1.0, 1.0);

        motor10.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motor9.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if(newKS != kS || newKV != kV || newKA != kA){

            kS = newKS;
            kV = newKV;
            kA = newKA;

            feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        }

        SmartDashboard.putNumber("TopRPM", getTopRPM());
        SmartDashboard.putNumber("BottomRPM", getBottomRPM());

    }

        // /** Spin shooter motors */
        // public void spin() {
        //     motor13.set(-0.23);//bottom, .20
        //     motor9.set(-0.30);//top, .25
        // }

        /** Stop shooter motors */
        //public void stop() {
        //    targetRPM = 0.0;
        //    motor10.stopMotor();
        //    motor9.stopMotor();
        //}
        public void stop() { // new stop
            targetRPM = 0.0;
            pid9.setSetpoint(0, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0);
            pid13.setSetpoint(0, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0);
        }

        public double getBottomRPM(){
            return motor10.getEncoder().getVelocity();
        }

        public double getTopRPM(){
            return motor9.getEncoder().getVelocity();
        }

        public boolean atSetpoint(double topTarget, double bottomTarget, double toleranceRPM) {
            return Math.abs(getTopRPM() - topTarget) < toleranceRPM
                && Math.abs(getBottomRPM() - bottomTarget) < toleranceRPM;
        }

        public double getDistanceToHub(Vision vision, int tagId1, int tagId2)
        {

            double dist1 = vision.getDistanceFromAprilTag(tagId1);
            double dist2 = vision.getDistanceFromAprilTag(tagId2);

            double distance;

            if (dist1 > 0 && dist2 > 0) {
                distance = (dist1 + dist2) / 2.0; // average
            } else if (dist1 > 0) {
                distance = dist1;
            } else if (dist2 > 0) {
                distance = dist2;
            } else {
                return -1; // no tag seen
            }

            double offset = 0.3;
            return distance + offset;
        }

/*    public double getDistanceToHub(){
        var result = camera1.getLatestResult();

        if(result.hasTargets()){
            for(PhotonTrackedTarget target : result.getTargets()){
                if(target.getFiducialId() == targetID){
                    Transform3d targetPose = target.getBestCameraToTarget();

                    double x = targetPose.getX();
                    double y = targetPose.getY();

                    double horizontalDistance = Math.sqrt(x*x + y*y);

                    return horizontalDistance ;
                }
            }
        }
        return -1;
    }
        */
}


