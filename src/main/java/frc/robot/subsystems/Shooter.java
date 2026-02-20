package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    
    private static TalonFX rightShooterMotor = new TalonFX(Constants.kRightShooterMotorID);

    private static TalonFX leftShooterMotor = new TalonFX(Constants.kLeftShooterMotorID);

    private static SparkMax hoodMotor = new SparkMax(Constants.kHoodMotorID, MotorType.kBrushless);
    private static SparkMaxConfig hoodConfig = new SparkMaxConfig();
    private static PIDController hoodPID = new PIDController(0.2, 0, 0);

    public Shooter() {
        rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
        
        leftShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        leftShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));

        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.smartCurrentLimit(5, 20);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void standardShooterSpeed() {
        rightShooterMotor.set(.75);
        leftShooterMotor.set(0.75);
    }

    private void shooterSpeedWithBangBang(double speed) {
        rightShooterMotor.set(speed);
        leftShooterMotor.set(speed);

        if((rightShooterMotor.getVelocity().getValueAsDouble() < 100) || leftShooterMotor.getVelocity().getValueAsDouble() < 100)
        {
            rightShooterMotor.set(speed + 0.15);
            leftShooterMotor.set(speed + 0.15);
        }
    }

    private void zeroShooterMotors() {
        rightShooterMotor.set(0);
        leftShooterMotor.set(0);
    }

    private void hoodForward() {
        hoodMotor.set(0.25);
    }

    private void hoodBackward() {
        hoodMotor.set(-0.25);
    }

    private void hoodAngle(double targetAngle) {
        hoodMotor.set(hoodPID.calculate(hoodMotor.getEncoder().getPosition(), targetAngle));
    }

    private void zeroHoodMotor() {
        hoodMotor.set(0);
    }

    //commands
    public Command standardShooterSpeedCommand() {
        return runEnd(() -> standardShooterSpeed(), () -> zeroShooterMotors());
    }

    public Command hoodForwardCommand() {
        return runEnd(() -> hoodForward(), () -> zeroHoodMotor());
    }

    public Command hoodBackwardCommand() {
        return runEnd(() -> hoodBackward(), () -> zeroHoodMotor());
    }
        
}