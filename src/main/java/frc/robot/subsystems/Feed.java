// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feed extends SubsystemBase {
  
  private static TalonFX feedMotor = new TalonFX(Constants.kFeedMotorID);
  private static TalonFXConfiguration feedConfig = new TalonFXConfiguration();
  
  /** Creates a new Feed. */
  public Feed() {
    feedConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    feedMotor.getConfigurator().apply(feedConfig);


  }

  private void feedSpeed() {
        feedMotor.set(0.75);
    }

  private void feedSpeedBackwards() {
        feedMotor.set(-0.75);
  }

    private void zeroFeedMotor() {
        feedMotor.set(0);
    }

    // private void fullIndexer() {
    //     feedMotor.set(0.5);
    //     mainBeltMotor.set(0.5);
    // }

    // private void zeroIndexerMotors() {
    //     feedMotor.set(0);
    //     mainBeltMotor.set(0);
    // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command feedSpeedCommand() {
    return runEnd(() -> feedSpeed(), () -> zeroFeedMotor());
  }

  public Command feedSpeedBackwardsCommand() {
    return runEnd(() -> feedSpeedBackwards(), () -> zeroFeedMotor());
  }
}
