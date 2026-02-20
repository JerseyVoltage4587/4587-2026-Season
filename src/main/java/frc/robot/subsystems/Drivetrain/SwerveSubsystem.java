// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem.
  public SwerveSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } */

  private final SwerveModule frontLeftModule = new SwerveModule(
    SwerveConstants.frontLeftDriveMotor,
    SwerveConstants.frontLeftTurnMotor,
    SwerveConstants.frontLeftDriveMotorReversed,
    SwerveConstants.frontLeftTurnMotorReversed,
    SwerveConstants.frontLeftAbsoluteEncoderPort,
    SwerveConstants.frontLeftAbsoluteEncoderReversed,
    SwerveConstants.frontLeftAbsoluteEncoderOffesetRad
  );
  
  private final SwerveModule frontRightModule = new SwerveModule(
    SwerveConstants.frontRightDriveMotor,
    SwerveConstants.frontRightTurnMotor,
    SwerveConstants.frontRightDriveMotorReversed,
    SwerveConstants.frontRightTurnMotorReversed,
    SwerveConstants.frontRightAbsoluteEncoderPort,
    SwerveConstants.frontRightAbsoluteEncoderReversed,
    SwerveConstants.frontRightAbsoluteEncoderOffesetRad
  );

  private final SwerveModule backLeftModule = new SwerveModule(
    SwerveConstants.backLeftDriveMotor,
    SwerveConstants.backLeftTurnMotor,
    SwerveConstants.backLeftDriveMotorReversed,
    SwerveConstants.backLeftTurnMotorReversed,
    SwerveConstants.backLeftAbsoluteEncoderPort,
    SwerveConstants.backLeftAbsoluteEncoderReversed,
    SwerveConstants.backLeftAbsoluteEncoderOffesetRad
  );

  private final SwerveModule backRightModule = new SwerveModule(
    SwerveConstants.backRightDriveMotor,
    SwerveConstants.backRightTurnMotor,
    SwerveConstants.backRightDriveMotorReversed,
    SwerveConstants.backRightTurnMotorReversed,
    SwerveConstants.backRightAbsoluteEncoderPort,
    SwerveConstants.backRightAbsoluteEncoderReversed,
    SwerveConstants.backRightAbsoluteEncoderOffesetRad
  );

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  
  //Position of Swerve Modules relative to center of robot
  final Translation2d frontLeftLocation = new Translation2d(RobotConstants.kRobotLengthMeters / 2, RobotConstants.kRobotWidthMeters / 2);
  final Translation2d frontRightLocation = new Translation2d(RobotConstants.kRobotLengthMeters / 2, -RobotConstants.kRobotWidthMeters / 2);
  final Translation2d backLeftLocation = new Translation2d(-RobotConstants.kRobotLengthMeters / 2, RobotConstants.kRobotWidthMeters / 2);
  final Translation2d backRightLocation = new Translation2d(-RobotConstants.kRobotLengthMeters / 2, -RobotConstants.kRobotWidthMeters / 2);

  //Kinematics object: ChassisSpeeds -> SwerveModuleStates
  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
    );
    
  SwerveModulePosition[] getModulePositions = {
    frontLeftModule.getPosition(), 
    frontRightModule.getPosition(), 
    backLeftModule.getPosition(), 
    backRightModule.getPosition()
  };

  SwerveModuleState[] getModuleStates = {
    frontLeftModule.getState(),
    frontRightModule.getState(),
    backLeftModule.getState(),
    backRightModule.getState()
  };
    
  private SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(getGyro()), getModulePositions);
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    kinematics, 
    getGyroToRotation2d(), 
    getModulePositions, 
    getPose(), 
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), 
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
  );

//   auton starts here!! :) woohoo
//   RobotConfig config;

//   private int counter = 0;
//   //Constructor
  public SwerveSubsystem() {
    super();

   new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroGyro();
        try {
        // config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          e.printStackTrace();
        }

//         // from pathplanner
//         AutoBuilder.configure(
//           this::getPose,
//           this::resetPose,
//           this::getCurrentSpeeds,
//           (speeds, feedforwards) -> driveRobotRelative(speeds, true),
//           new PPHolonomicDriveController(
//             new PIDConstants(0.1, 0.0, 0.0),
//             new PIDConstants(0.1, 0.0, 0.0)
//           ),
//           config,
//           () -> {
//             var alliance = DriverStation.getAlliance();
//             if (alliance.isPresent()) {
//               return alliance.get() == DriverStation.Alliance.Red;
//             }
//             return false;
//           },
//           this
//         );
      } catch (Exception e) {
      }
   }).start();

  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void backwardsGyro() {
    gyro.setAngleAdjustment(180);
  }

  public double getGyro() {
    return -Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getGyroToRotation2d() {
    return Rotation2d.fromDegrees(getGyro());
  }

  public void zeroModules() {
    frontLeftModule.zeroMotors();
    frontRightModule.zeroMotors();
    backLeftModule.zeroMotors();
    backRightModule.zeroMotors();
  }

  public void setChassisSpeeds(ChassisSpeeds speeds)
  {

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, false);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean planned) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, planned ? RobotConstants.kMaxSpeed : 1.0);
    
    frontLeftModule.setDesiredState(desiredStates[0], planned);
    frontRightModule.setDesiredState(desiredStates[1], planned);
    backLeftModule.setDesiredState(desiredStates[2], planned);
    backRightModule.setDesiredState(desiredStates[3], planned);
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    if (pose != null) {
      odometer.resetPose(pose);
    }
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
      kinematics.toChassisSpeeds(
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()),
      getGyroToRotation2d());

  }
  public void driveRobotRelative(ChassisSpeeds speeds) {
    driveRobotRelative(speeds, false);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, boolean planned) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds), planned);
  }

  public void drive(Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction,
      Supplier<Double> thetaFunction, Supplier<Boolean> fieldOrientedFunction)
    {
      ChassisSpeeds spds;

      if (!fieldOrientedFunction.get()) {
        spds = ChassisSpeeds.fromFieldRelativeSpeeds(
          MathUtil.applyDeadband(xSpeedFunction.get(), RobotConstants.kDeadBand),
          MathUtil.applyDeadband(ySpeedFunction.get(), RobotConstants.kDeadBand),
          MathUtil.applyDeadband(thetaFunction.get(), RobotConstants.kDeadBand),
          getGyroToRotation2d()
        );
      } else {
        spds = ChassisSpeeds.fromRobotRelativeSpeeds(
          MathUtil.applyDeadband(xSpeedFunction.get() * 0.25, RobotConstants.kDeadBand),
          MathUtil.applyDeadband(ySpeedFunction.get() * 0.25, RobotConstants.kDeadBand),
          MathUtil.applyDeadband(thetaFunction.get() * 0.5, RobotConstants.kDeadBand), 
          getGyroToRotation2d()
        );
      }
      
      setModuleStates(
        kinematics.toSwerveModuleStates(spds)
      );
      SmartDashboard.putString("DriveSpds", spds.toString());
      SmartDashboard.putBoolean("FieldOrient", !fieldOrientedFunction.get());
    }

  public double angleDirection(double angle) {
    if ((angleDistance(angle)) < 180) {
      return angle/180;
    } else {
      return -(angle/180);
    }
  }

  public double angleDistance(double angle) {
    return Rotation2d.fromDegrees(angle).minus(getGyroToRotation2d()).getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometer.update(getGyroToRotation2d(), getModulePositions);
    
    SmartDashboard.putNumber("Gyro Degrees", getGyro());
    SmartDashboard.putNumber("Front Left Offset Rad", frontLeftModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Right Offset Rad", frontRightModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Left Offset Rad", backLeftModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Back Right Offset Rad", backRightModule.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Front Left Motor Rad", frontLeftModule.getTurnPosition());
    SmartDashboard.putNumber("Front Right Motor Rad", frontRightModule.getTurnPosition());
    SmartDashboard.putNumber("Back Left Motor Rad", backLeftModule.getTurnPosition());
    SmartDashboard.putNumber("Back Right Motor Rad", backRightModule.getTurnPosition());
    SmartDashboard.putNumber("Gyro", getGyro());   

    // if (this.getCurrentCommand() != null)
    //   SmartDashboard.putData("Swerve Cmd", this.getCurrentCommand());

    // SmartDashboard.putString("Swerve Spds",this.getCurrentSpeeds().toString());
  }

  

  //Command Methods
  public Command DriveCommand(Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction,
    Supplier<Double> thetaFunction, Supplier<Boolean> fieldOrientedFunction)
  {
    return runEnd(() -> drive(xSpeedFunction, ySpeedFunction, thetaFunction, fieldOrientedFunction), () -> zeroModules());
  }

  public Command ToAngleCommand(Supplier<Double> angle)
  {
    return DriveCommand(() -> 0.0, () -> 0.0, () -> angleDirection(angle.get()), () -> false)
          .until(() -> angleDistance(angle.get()) < 1);
  }

  // public Command ForwardAtAngleCommand(Supplier<Double> angle)
  // {
  //   return ToAngleCommand(angle).andThen(DistanceToReefCommand());
  // }
  
  // public Command DistanceToReefCommand() {
  //   return DriveCommand(() -> reefPercentage(), () -> 0.0, () -> 0.0, () -> true)
  //           .until(() -> LLTA > 75);
  // }
  // public Command ForwardAtAngleCommand(Supplier<Double> angle, double seconds)
  // {
  //   return ToAngleCommand(angle).andThen(DriveCommand( () -> 0.1, () -> 0.0, () -> 0.0, () -> false).withTimeout(seconds));
  // }

  public Command ZeroGyroCommand() {
    return runOnce(() -> zeroGyro());
  }


}
