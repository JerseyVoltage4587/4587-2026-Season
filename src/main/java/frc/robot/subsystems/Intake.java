package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static TalonFX intakeBallMotor = new TalonFX(Constants.kIntakeBallMotorID);
    private static TalonFXConfiguration intakeBallConfig = new TalonFXConfiguration();

    private static SparkMax releaseIntakeMotor = new SparkMax(Constants.kReleaseIntakeMotorID, MotorType.kBrushless);
    private static SparkMaxConfig releaseIntakeConfig = new SparkMaxConfig();
    
    public Intake() {

        intakeBallConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        intakeBallMotor.getConfigurator().apply(intakeBallConfig);


        releaseIntakeConfig.idleMode(IdleMode.kCoast);
        releaseIntakeConfig.inverted(true);
        // releaseIntakeConfig.smartCurrentLimit(2, 30);
        releaseIntakeMotor.configure(releaseIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    
    @Override
    public void periodic() {

    }

    private void intakeBallMotorSpeed() {
        intakeBallMotor.set(-0.5);
    }

    private void outtakeBallMotorSpeed() {
        intakeBallMotor.set(0.5);
    }
    
    private void zeroIntakeBallMotor() {
        intakeBallMotor.set(0);
    }

    private void bringReleaseIntakeOut() {
        releaseIntakeMotor.set(0.4);
    }

    private void bringReleaseIntakeIn() {
        releaseIntakeMotor.set(-0.4);
    }

    private void bringIntakeInSlow() {
        releaseIntakeMotor.set(-0.2);
    }

    private void zeroReleaseIntake() {
        releaseIntakeMotor.set(0);
    }

    //commands

    public Command RunIntakeBallCommand() {
        return runEnd(() -> intakeBallMotorSpeed(), () -> zeroIntakeBallMotor()).withName("RunIntake");
    }

    public Command RunOuttakeBallCommand() {
        return runEnd(() -> outtakeBallMotorSpeed(), () -> zeroIntakeBallMotor());
    }
    
    public Command ZeroIntakeBallMotorCommand() {
        return run(() -> zeroIntakeBallMotor()).withName("ZeroIntake");
    }

    public Command bringReleaseIntakeOutCommand() {
        return runEnd(() -> bringReleaseIntakeOut(), () -> zeroReleaseIntake());
    }

    public Command bringReleaseIntakeInCommand() {
        return runEnd(() -> bringReleaseIntakeIn(), () -> zeroReleaseIntake());
    }

    public Command bringIntakeInSlowCommand() {
        return runEnd(() -> bringIntakeInSlow(), () -> zeroReleaseIntake());
    }
}