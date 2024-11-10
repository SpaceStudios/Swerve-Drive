// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import javax.sound.sampled.ReverbType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Subsystems.Drivetrain_Swerve.SwerveIO.SwerveData;

/** Add your docs here. */
public class SwerveModule {
    double CurrentDistance;
    SwerveIO io;
    SwerveData mainData;
    SwerveDriveKinematics kinematics;

    public SwerveModule(double Distance, int DriveID, int SteerID) {
        CurrentDistance = Distance;
        io = new SwerveIO_SIM(DriveID, SteerID);
        mainData = new SwerveData();
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition newPos = new SwerveModulePosition(getDriveDistance(),getRotation());
        return newPos;
    }

    public void setDriveVolts(double volts) {
        io.setDriveVolts(volts);
    }

    public void setSteerAngle(double angleRad) {
        io.setSteerAngle(angleRad);
    }

    public double getDriveDistance() {
        return io.getDistance();
    }

    public Rotation2d getRotation() {
        return io.getRotation();
    }

    public SwerveModuleState getCurrentState() {
        io.getData(mainData);
        return new SwerveModuleState(mainData.DriveVelocity,Rotation2d.fromRadians(mainData.SteerPosition*(2*Math.PI)));
    }
}
