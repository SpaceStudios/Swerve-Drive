// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class shooterIO_SparkMax implements shooterIO {
    CANSparkMax shootMotor;
    CANSparkMax launchMotor;

    RelativeEncoder shootEncoder;
    RelativeEncoder launchEncoder;

    public shooterIO_SparkMax(int ShooterID, int LauncherID) {
        shootMotor = new CANSparkMax(ShooterID, MotorType.kBrushless);
        launchMotor = new CANSparkMax(LauncherID, MotorType.kBrushless);

        shootEncoder = shootMotor.getEncoder();
        launchEncoder = launchMotor.getEncoder();

        shootMotor.restoreFactoryDefaults();
        shootMotor.setCANTimeout(250);
        REVLibError shootMotorSucess = REVLibError.kUnknown;
        while (shootMotorSucess != REVLibError.kOk) {
            shootMotorSucess = shootMotor.burnFlash();
        }

        REVLibError launchMotorSucess = REVLibError.kUnknown;
        while (launchMotorSucess != REVLibError.kOk) {
            launchMotorSucess = launchMotor.burnFlash();
        }
    }

    @Override
    public void setShooterVolts(double volts) {
        shootMotor.setVoltage(volts);
    }

    @Override
    public void setLauncherVolts(double volts) {
        launchMotor.setVoltage(volts);
    }

    @Override
    public double getShooterSpeed() {
        return shootEncoder.getVelocity()/60.0;
    }

    @Override
    public void motorUpdate() {
        
    }

}
