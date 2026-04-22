// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule { // class swervemodule
  
  //instatntiating a bunch of variables

  private TalonFX driveMotor;
  //private TalonFX turnMotor;
  private SparkMax turnMotor;

  private SparkMaxConfig turnMotorConfig;
  private TalonFXConfiguration driveMotorConfig;

  private PIDController turnPIDController;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  
  //Constructor (giving variables values)
  public SwerveModule(int driveMotorPort, int turnMotorPort, boolean driveMotorReversed, boolean turnMotorReversed,
  int absoluteEncoderID, boolean absoluteEncoderReversed, double absoluteEncoderOffset) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoder = new CANcoder(absoluteEncoderID);

    
    driveMotor = new TalonFX(driveMotorPort);
    turnMotor = new SparkMax(turnMotorPort, MotorType.kBrushless);

    turnMotorConfig = new SparkMaxConfig();
    driveMotorConfig = new TalonFXConfiguration();
    
    
    if (turnMotorReversed) {
      turnMotorConfig.inverted(true);
    }

    if (driveMotorReversed) {
      driveMotorConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    }

    driveMotorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    driveMotorConfig.CurrentLimits.withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
    driveMotor.getConfigurator().apply(driveMotorConfig);
    
    turnMotorConfig.idleMode(IdleMode.kBrake);
    turnMotorConfig.smartCurrentLimit(40);
    turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    turnPIDController = new PIDController(SwerveConstants.kSwerveP, SwerveConstants.kSwerveI, SwerveConstants.kSwerveD);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public double getDrivePosition() {
    return driveMotor.getRotorPosition().getValueAsDouble()
    / RobotConstants.kDriveMotorGearRatio * Math.PI * RobotConstants.kWheelDiameter;
  }

  public double getTurnPosition() {
    return turnMotor.getEncoder().getPosition()
    / RobotConstants.kTurnMotorGearRatio * 2 * Math.PI;
  }

  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValueAsDouble()
    / RobotConstants.kDriveMotorGearRatio * Math.PI * RobotConstants.kWheelDiameter;
  }

  public double getTurnVelocity() {
    return turnMotor.getEncoder().getPosition()
    / RobotConstants.kDriveMotorGearRatio * Math.PI * RobotConstants.kWheelDiameter;
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveMotor.setPosition(0);
    turnMotor.getEncoder().setPosition(getAbsoluteEncoderRad()*RobotConstants.kTurnMotorGearRatio/(2*Math.PI));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
  }

  public void zeroMotors() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState desiredState)
  {
    setDesiredState(desiredState, false);
  }


  public void setDesiredState(SwerveModuleState desiredState, boolean planned) {
    desiredState.optimize(getState().angle);
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      driveMotor.set(0);
    }
    else
    {
      driveMotor.set(desiredState.speedMetersPerSecond / (planned ? RobotConstants.kMaxSpeed : 1));
    }
    
    turnMotor.set(turnPIDController.calculate(getTurnPosition(), desiredState.angle.getRadians()));
    // SmartDashboard.putString("SwerveModule[" + absoluteEncoder.getDeviceID() + "] state", desiredState.toString());
  }
    
}
