// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveIO_SIM implements SwerveIO {
    DCMotorSim DriveMotor;
    DCMotorSim SteerMotor;

    public SwerveIO_SIM(int DriveID, int SteerID) {
        DriveMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.driveRatio, Constants.driveMOI);
        SteerMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.steerRatio, Constants.steerMOI);
    }

    @Override
    public void setDriveVolts(double volts) {
        DriveMotor.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setSteerAngle(double angle) {
        SteerMotor.setInputVoltage(MathUtil.clamp(angle*24, -12, 12));
    }

    @Override
    public void getData(SwerveData data) {
        data.DriveOutput = DriveMotor.getCurrentDrawAmps();
        data.DrivePosition = DriveMotor.getAngularPositionRotations();
        data.DriveVelocity = DriveMotor.getAngularVelocityRPM()*60.0;
        data.SteerPosition = SteerMotor.getAngularPositionRad();
        data.SteerOutout = SteerMotor.getCurrentDrawAmps();
    }

    @Override
    public double getDistance() {
        return DriveMotor.getAngularPositionRotations();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(SteerMotor.getAngularPositionRad());
    }
}
