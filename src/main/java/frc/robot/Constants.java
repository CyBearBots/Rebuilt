// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.2;
    public static final double RIGHT_X_DEADBAND = 0.2;
    public static final double TURN_CONSTANT    = 5;
  }

  public static class ArmConstants
  {
    public static final double kPUp = 0.05; //.042
    public static final double kPDown = 0.04;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double armUpLimit = 24.3;
    public static final double armDownLimit = 0.0;

  }

  /*public static class IntakeConstants
  {
      public static final int intakeMotorId = 13;
      
      public static final double kP = 0.002;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.12; // feedforward, same as shooter

      public static final double spinRPM = -5400;     // tune these
      public static final double slowSpinRPM = -5400;
  }*/

  public static class ShooterConstants
  {

    // TOP RPM > BOTTOM RPM = TOPSPIN, SHALLOWER LAUNCH ANGLE
    // BOTTOM RPM > TOP RPM = BACKSPIN, STEEPER LAUNCH ANGLE

    public static final double topRPMDefault = -1060; //-1060
    public static final double spinRatio = -1.4; // Above 1 = backspin, Below 1 = topspin (ignoring negative sign, that's just for motor direction)
    public static final double spinRationUnder5Ft = -1.6;

    public static double kP = 0.0002;
    public static double kI = 0;
    public static double kD = 0;

    public static double kS = 0;
    public static double kV = 0.12;
    public static double kA = 0;

    public static final double rpmTolerance = 75;

    public static final double passRPM = -2100;
    public static final double passSpinRatio = -1.1;
  }

  public static class VisionConstants
  {
    
  }
}
