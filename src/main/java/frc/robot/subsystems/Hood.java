package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

//big class full of stuff
public class Hood extends SubsystemBase{
    
    //variables of class (motor, confic, PID controller)
    private static SparkMax hoodMotor = new SparkMax(Constants.kHoodMotorID, MotorType.kBrushless);
    private static SparkMaxConfig hoodConfig = new SparkMaxConfig();
    private static PIDController hoodPIDController = new PIDController(
        HoodConstants.kHoodP, HoodConstants.kHoodI, HoodConstants.kHoodD
    );

    //does stuff to the variables
    public Hood() {
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.inverted(true);
        // hoodConfig.smartCurrentLimit(5, 20);
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        hoodMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Encoder", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Current", hoodMotor.getOutputCurrent());
        hoodZeroCurrentBased();
    }
    // Hood methods below

    //Manual Hood Adjustment Functions (goes forward and backwards)
    private void hoodForward() {
        hoodMotor.set(0.1);
    }

    private void hoodBackward() {
        hoodMotor.set(-0.1);
    }

    private void hoodZeroCurrentBased() {
        if (hoodMotor.getEncoder().getPosition() < 2 && hoodMotor.getOutputCurrent() > 30) {
            zeroHoodEncoder();
        }
    }
  
    private void hoodMaxCurrentBased() {
        if (hoodMotor.getEncoder().getPosition() > 4 && hoodMotor.getOutputCurrent() > 30) {
            hoodMotor.getEncoder().setPosition(6.35);
        }
    }

    //Automatic Hood alignment based on distance

    //this part gives a target angle for the hood, and uses PID to get there
    private void hoodAngle(double targetAngle) {
        hoodMotor.set(hoodPIDController.calculate(hoodMotor.getEncoder().getPosition(), targetAngle));
    }


    //so BASICALLY this thing actually does the PID: 
    private void goToDegrees(double deg) {
        hoodAngle((deg / 45) * HoodConstants.kMaxHoodEncoderValue);

    }

    private void zeroHoodMotor() {
        hoodMotor.set(0);
    }

    private void zeroHoodEncoder() {
        hoodMotor.getEncoder().setPosition(0);
    }

    // Hood commands below

    public Command zeroHoodCommand() {
        return runOnce(() -> zeroHoodEncoder());
    }

    public Command hoodForwardCommand() {
        return runEnd(() -> hoodForward(), () -> zeroHoodMotor());
    }

    public Command hoodBackwardCommand() {
        return runEnd(() -> hoodBackward(), () -> zeroHoodMotor());
    }

   public Command hoodGoToDegreesCommand(DoubleSupplier deg) {
        return runEnd(() -> goToDegrees(deg.getAsDouble()), () -> zeroHoodMotor());
   }

   public Command hoodGoToPosCommand(DoubleSupplier pos) {
        return runEnd(() -> hoodAngle(pos.getAsDouble()), () -> zeroHoodMotor());
   }
}