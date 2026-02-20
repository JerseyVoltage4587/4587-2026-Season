// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {  

public static class SwerveConstants {
    public static final int frontLeftDriveMotor = 1;
    public static final int frontLeftTurnMotor = 0;
    public static final boolean frontLeftDriveMotorReversed = true;
    public static final boolean frontLeftTurnMotorReversed = true;
    public static final int frontLeftAbsoluteEncoderPort = 17;
    public static final double frontLeftAbsoluteEncoderOffesetRad = -1.247; //-1.963567;
    public static final boolean frontLeftAbsoluteEncoderReversed = false;

    public static final int frontRightDriveMotor = 3;
    public static final int frontRightTurnMotor = 2;
    public static final boolean frontRightDriveMotorReversed = false;
    public static final boolean frontRightTurnMotorReversed = true;
    public static final int frontRightAbsoluteEncoderPort = 18;
    public static final double frontRightAbsoluteEncoderOffesetRad = 2.486;
    public static final boolean frontRightAbsoluteEncoderReversed = false;

    public static final int backLeftDriveMotor = 7;
    public static final int backLeftTurnMotor = 6;
    public static final boolean backLeftDriveMotorReversed = true;
    public static final boolean backLeftTurnMotorReversed = false;
    public static final int backLeftAbsoluteEncoderPort = 16;
    public static final double backLeftAbsoluteEncoderOffesetRad = -2.729;
    public static final boolean backLeftAbsoluteEncoderReversed = false;

    public static final int backRightDriveMotor = 5;
    public static final int backRightTurnMotor = 4;
    public static final boolean backRightDriveMotorReversed = false;
    public static final boolean backRightTurnMotorReversed = true;
    public static final int backRightAbsoluteEncoderPort = 19;
    public static final double backRightAbsoluteEncoderOffesetRad = -3.836;
    public static final boolean backRightAbsoluteEncoderReversed = false;
    
    public static final double kSwerveP = 0.2;
    public static final double kSwerveI = 0;
    public static final double kSwerveD = 0;
  }

  public static class RobotConstants {
    
    public static final double kRobotWidthMeters = Units.inchesToMeters(24);
    public static final double kRobotLengthMeters = Units.inchesToMeters(30);
  
    public static final double kMaxSpeed = 4.5;
    public static final double kMaxAngularSpeed = Math.PI;
    public static final double kDeadBand = 0.05;
    public static final double kSwerveMaxAcceleration = 0.5;
    public static final double kSwerveMaxAngularAcceleration = 0.5;

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurnMotorGearRatio = 150/7;
    
  }

  //IDs
  public static final int kTurretMotorID = 8;

  //intake
  public static final int kIntakeBallMotorID = 9;
  public static final int kReleaseIntakeMotorID = 16;

  //indexer
  // public static final int kSpindexerMotorID = 10;
  public static final int kFromIntakeMotorID = 10;
  public static final int kToShooterMotorID = 11;

  //shoter
  public static final int kLeftShooterMotorID = 12;
  public static final int kRightShooterMotorID = 13;
  public static final int kHoodMotorID = 14;

  //climber
  public static final int kClimberMotorID = 15;
}
