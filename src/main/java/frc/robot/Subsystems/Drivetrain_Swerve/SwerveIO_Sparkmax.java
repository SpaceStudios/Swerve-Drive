// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveIO_Sparkmax implements SwerveIO {
    
    CANSparkMax DriveMotor;
    CANSparkMax SteerMotor;
    
    RelativeEncoder driveEncoder;
    RelativeEncoder steerEncoder;

    PIDController drivePID;
    PIDController steerPID;
    public SwerveIO_Sparkmax(int DriveID, int SteerID) {
        DriveMotor = new CANSparkMax(DriveID, MotorType.kBrushless);
        SteerMotor = new CANSparkMax(SteerID, MotorType.kBrushless);
        
        driveEncoder = DriveMotor.getEncoder();
        steerEncoder = SteerMotor.getEncoder();

        drivePID = new PIDController(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive);
        steerPID = new PIDController(Constants.kPSteer, Constants.kISteer, Constants.KDSteer);
    }

    @Override
    public void setDriveVolts(double volts) {
        DriveMotor.setVoltage(volts);
    }

    @Override
    public void setDriveSpeed(double rps) {
        DriveMotor.setVoltage(drivePID.calculate(rps));
    }

    @Override
    public void setSteerAngle(double angle) {
        SteerMotor.setVoltage(steerPID.calculate(steerEncoder.getPosition()*(2*Math.PI), angle));
    }

    @Override
    public double getDistance() {
        return driveEncoder.getVelocity();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(steerEncoder.getPosition());
    }

    @Override
    public void getData(SwerveData data) {
        data.DriveOutput = DriveMotor.getAppliedOutput();
        data.SteerOutout = SteerMotor.getAppliedOutput();
        
        data.DrivePosition = driveEncoder.getPosition();
        data.DriveVelocity = driveEncoder.getVelocity();
        data.SteerPosition = steerEncoder.getPosition();
    }
}
