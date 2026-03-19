package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    
    private static TalonFX rightShooterMotor = new TalonFX(Constants.kRightShooterMotorID);
    private static TalonFX leftShooterMotor = new TalonFX(Constants.kLeftShooterMotorID);
    private double shooterSpeed = 0;
    private static PIDController shooterPIDController = new PIDController(
        ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD
    );

    public Shooter() {
        rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
        
        leftShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        leftShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    }

    //Sets Shooter Speed to a Constant 
    private void standardShooterSpeed() {
        rightShooterMotor.set(shooterSpeed);
        leftShooterMotor.set(shooterSpeed);
    }

    //Variable Shooter Speeds for Standard Method
    private void fasterShoot() {
        shooterSpeed += 0.05;
    }

    private void slowerShoot() {
        shooterSpeed -= 0.05;
    }

    public double displayShooter() {
        return shooterSpeed;
    }

    public double displayShooterGet() {
        return (leftShooterMotor.get() + rightShooterMotor.get()) / 2;
    }

    //Shooter Speed Method with a Bang Bang Controller to combat speed dropping
    private void shooterSpeedWithBangBang(double speed) {        
        rightShooterMotor.set(speed);   
        leftShooterMotor.set(speed);

        if((rightShooterMotor.getVelocity().getValueAsDouble() < 100) || leftShooterMotor.getVelocity().getValueAsDouble() < 100)
        {
            rightShooterMotor.set(speed + 0.15);
            leftShooterMotor.set(speed + 0.15);
        }
    }

    //Shooter Speed Method with a PID Controller to combat speed dropping
    private void shooterSpeedWithPID(double spd) {
        leftShooterMotor.set(shooterPIDController.calculate(leftShooterMotor.get(), spd));
        rightShooterMotor.set(shooterPIDController.calculate(rightShooterMotor.get(), spd));
    }

    private void zeroShooterMotors() {
        rightShooterMotor.set(0);
        leftShooterMotor.set(0);
    }


    //commands
    public Command standardShooterSpeedCommand() {
        return runEnd(() -> standardShooterSpeed(), () -> zeroShooterMotors());
    }
    
    public Command fasterShootCommand() {
        return runOnce(() -> fasterShoot());
    }
    
    public Command slowerShootCommand() {
        return runOnce(() -> slowerShoot());
    }

    public Command shooterSpeedWithPIDCommand() {
        return runEnd(() -> shooterSpeedWithPID(shooterSpeed), () -> zeroShooterMotors());
    }

}