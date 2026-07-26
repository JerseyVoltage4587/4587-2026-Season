// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final int frontLeftDriveMotor = 2;
    public static final int frontLeftTurnMotor = 1;
    public static final boolean frontLeftDriveMotorReversed = false;
    public static final boolean frontLeftTurnMotorReversed = false;
    public static final int frontLeftAbsoluteEncoderPort = 17;
    public static final double frontLeftAbsoluteEncoderOffesetRad = -6.714;
    public static final boolean frontLeftAbsoluteEncoderReversed = false;

    public static final int frontRightDriveMotor = 4;
    public static final int frontRightTurnMotor = 3;
    public static final boolean frontRightDriveMotorReversed = true;
    public static final boolean frontRightTurnMotorReversed = true;
    public static final int frontRightAbsoluteEncoderPort = 18;
    public static final double frontRightAbsoluteEncoderOffesetRad = 1.140;
    public static final boolean frontRightAbsoluteEncoderReversed = false;

    public static final int backLeftDriveMotor = 8;
    public static final int backLeftTurnMotor = 7;
    public static final boolean backLeftDriveMotorReversed = true;
    public static final boolean backLeftTurnMotorReversed = true;
    public static final int backLeftAbsoluteEncoderPort = 16;
    public static final double backLeftAbsoluteEncoderOffesetRad = 0.356;
    public static final boolean backLeftAbsoluteEncoderReversed = false;

    public static final int backRightDriveMotor = 6;
    public static final int backRightTurnMotor = 5;
    public static final boolean backRightDriveMotorReversed = true;
    public static final boolean backRightTurnMotorReversed = true;
    public static final int backRightAbsoluteEncoderPort = 19;
    public static final double backRightAbsoluteEncoderOffesetRad = -1.101;
    public static final boolean backRightAbsoluteEncoderReversed = false;
    
    public static final double kSwerveP = 0.2;
    public static final double kSwerveI = 0;
    public static final double kSwerveD = 0;
  }

  public static class RobotConstants {
    
    public static final double kRobotWidthMeters = Units.inchesToMeters(24);
    public static final double kRobotLengthMeters = Units.inchesToMeters(30);
  
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = 540.0;
    public static final double kDeadBand = 0.025;
    public static final double kSwerveMaxAcceleration = 3.0;
    public static final double kSwerveMaxAngularAcceleration = 720.0;

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurnMotorGearRatio = 150/7;
    
  }

  public static final class ShooterConstants {

    public static final double kMinShooterMotorSpeed = 0.55;
    public static final double kMaxShooterMotorSpeed = 0.85;
    public static final double kMinMaxShooterSpeedDifference = kMaxShooterMotorSpeed - kMinShooterMotorSpeed;

    public static final double kShooterP = 0;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
  }

  public static final class HoodConstants {

    public static final double kMaxHoodEncoderValue = 6.35;
    
    public static final double kHoodP = 0.18;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
  }

  public static final class TurretConstants {
    
    public static final double kTurretToRobotFrontOffset = 180; 
    public static final double kMaxTurretEncoderValue = 20.761;
    public static final double kMinTurretEncoderValue = -19.738;
    
    public static final double kTurretP = 1000;
    public static final double kTurretI = 0;
    public static final double kTurretD = 0;
  }

  public static final Translation2d kTeamHubCoord = new Translation2d(4.616, 4.035);
  public static final Translation2d kTeamDepotTrenchCoord = new Translation2d(4.616, 0.641);
  public static final Translation2d kTeamOutpostTrenchCoord = new Translation2d(4.616, 7.422);
  public static final Translation2d kOppHubCoord = new Translation2d(11.906, 4.035);
  public static final Translation2d kOppDepotTrenchCoord = new Translation2d(11.906, 7.422);
  public static final Translation2d kOppOutpostTrenchCoord = new Translation2d(11.906, 0.641);
  public static final double kMaxDistanceInMeters = 6.231;

  public static final double kShooterRPSMax = 72;

  //IDs
  public static final int kTurretMotorID = 20;

  //intake
  public static final int kIntakeBallMotorID = 32;
  public static final int kReleaseIntakeMotorID = 33;

  //indexer
  public static final int kMainBeltMotorID = 30;
  public static final int kFeedMotorID = 31;

  //shoter
  public static final int kLeftShooterMotorID = 21;
  public static final int kRightShooterMotorID = 22;
  public static final int kHoodMotorID = 23;

  //climber
  // public static final int kClimberMotorID = 40;
}
