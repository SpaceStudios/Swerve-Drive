// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.drivetrainConstants;

/** Add your docs here. */
public class SwerveIO_Forte implements SwerveIO{
    TalonFX driveTalon;
    CANSparkMax steerMotor;
    PIDController drivePID;
    SparkPIDController steerPID;
    RelativeEncoder steerEncoder;
    public SwerveIO_Forte(int DriveID, int SteerID) {
        driveTalon = new TalonFX(DriveID);
        steerMotor = new CANSparkMax(SteerID, MotorType.kBrushless);
        drivePID = new PIDController(drivetrainConstants.kPDrive, drivetrainConstants.kIDrive, drivetrainConstants.kDDrive);
        steerPID = steerMotor.getPIDController();
        steerPID.setP(drivetrainConstants.kPSteer);
        steerPID.setI(drivetrainConstants.kISteer);
        steerPID.setD(drivetrainConstants.kDSteer);
        
        // Can Spark Max Settings
        steerMotor.restoreFactoryDefaults();
        steerMotor.setCANTimeout(250);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerEncoder = steerMotor.getEncoder();

        REVLibError SteerMotorSucess = REVLibError.kUnknown;
        while (SteerMotorSucess != REVLibError.kOk) {
            SteerMotorSucess = steerMotor.burnFlash();
        }
    }

    @Override
    public void setDriveVolts(double volts) {
        driveTalon.setVoltage(volts);
    }

    @Override
    public void setDriveSpeed(double rps) {
        driveTalon.setVoltage(drivePID.calculate(driveTalon.getVelocity().getValueAsDouble(),rps));
    }

    @Override
    public void setSteerAngle(double angle) {
        steerPID.setReference(angle*(Math.PI*2), ControlType.kPosition);
    }

    @Override
    public double getDistance() {
        return driveTalon.getPosition().getValueAsDouble();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(steerEncoder.getPosition());
    }

    @Override
    public void getData(SwerveData data) {
        data.DriveOutput = driveTalon.getTorqueCurrent().getValueAsDouble();
        data.SteerOutout = steerMotor.getAppliedOutput();
        data.DriveVelocity = driveTalon.getVelocity().getValueAsDouble();
        data.DrivePosition = driveTalon.getPosition().getValueAsDouble();
        data.SteerPosition = steerEncoder.getPosition();
    }
}
