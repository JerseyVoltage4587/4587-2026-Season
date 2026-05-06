package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    
    private static TalonFX rightShooterMotor = new TalonFX(Constants.kRightShooterMotorID);
    private static TalonFX leftShooterMotor = new TalonFX(Constants.kLeftShooterMotorID);
    private static TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    private double shooterSpeed = 0.55;
    private static PIDController shooterPIDController = new PIDController(
        ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD
    );

    public Shooter() {
        // rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(NeutralModeValue.Coast));
        // rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
        
        leftConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        leftConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        leftConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(0.1);
        
        rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Speed Variable", shooterSpeed);
        SmartDashboard.putNumber("Shooter Motor Speed", leftShooterMotor.get());
        SmartDashboard.putNumber("Shooter Speed Velocity", leftShooterMotor.getVelocity().getValueAsDouble());

    }

    //Sets Shooter Speed to a Constant 
    private void standardShooterSpeed() {
        // rightShooterMotor.set(shooterSpeed);
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
    private void shooterSpeedWithBangBang() {        
        // rightShooterMotor.set(shooterSpeed);   
        

        if((Math.abs(leftShooterMotor.getVelocity().getValueAsDouble()) < ((Constants.kShooterRPSMax - 8.8) * shooterSpeed)))
        {
            // rightShooterMotor.set(shooterSpeed + 0.1);
            leftShooterMotor.set(shooterSpeed + 0.15);
        } else {
            leftShooterMotor.set(shooterSpeed);
        }
    }

    //Shooter Speed Method with a Bang Bang Controller to combat speed dropping
    private void shooterSpeedWithBangBang(double baselinespd) {        
        // rightShooterMotor.set(shooterSpeed);   
        

        if((Math.abs(leftShooterMotor.getVelocity().getValueAsDouble()) < ((Constants.kShooterRPSMax - 8.8) * shooterSpeed)))
        {
            // rightShooterMotor.set(shooterSpeed + 0.1);
            leftShooterMotor.set(baselinespd);
        } else {
            leftShooterMotor.set(shooterSpeed);
        }
    }

    //Shooter Speed Method with a PID Controller to combat speed dropping
    private void shooterSpeedWithPID(double spd) {
        leftShooterMotor.set(shooterPIDController.calculate(leftShooterMotor.get(), spd));
        rightShooterMotor.set(shooterPIDController.calculate(rightShooterMotor.get(), spd));
    }

    // private void autoShooterSpeedWithPID(double currentDist) {
    //     leftShooterMotor.set(ShooterConstants.kMinMaxShooterSpeedDifference * (currentDist / Constants.kMaxDistanceInMeters) + ShooterConstants.kMinShooterMotorSpeed);
    // }

    private void zeroShooterMotors() {
        rightShooterMotor.set(0);
        leftShooterMotor.set(0);
    }


    //commands
    public Command bangBangCommand() {
        return runEnd(() -> shooterSpeedWithBangBang(), () -> zeroShooterMotors());
    }

    public Command bangBangCommand(DoubleSupplier spd) {
        return runEnd(() -> shooterSpeedWithBangBang(spd.getAsDouble()), () -> zeroShooterMotors());
    }

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