// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.robotConstants;
import frc.robot.Subsystems.Drivetrain_Swerve.SwerveIO.SwerveData;

/** Add your docs here. */
public class SwerveModule {
    double CurrentDistance;
    SwerveIO io;
    SwerveData mainData;
    SwerveDriveKinematics kinematics;
    int SetDriveID;
    int SetSteerID;

    public SwerveModule(double Distance, int DriveID, int SteerID) {
        CurrentDistance = Distance;
        switch (robotConstants.currentMode) {
            case SIM:
                io = new SwerveIO_SIM(DriveID, SteerID);
                break;
            case REAL:
                io = new SwerveIO_Sparkmax(DriveID, SteerID);
                break;
            case REPLAY:
                io = new SwerveIO_SIM(DriveID, SteerID);
                break;
        }
        mainData = new SwerveData();
        SetDriveID = DriveID;
        SetSteerID = SteerID;
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition newPos = new SwerveModulePosition(getDriveDistance(),getRotation());
        return newPos;
    }

    public void setDriveVolts(double volts) {
        io.setDriveVolts(volts);
    }

    public void setSteerAngle(Rotation2d angleRad) {
        Logger.recordOutput("Steer Input"+SetSteerID, angleRad);
        if (angleRad == -Math.PI || angleRad == Math.PI) {
            angleRad = Math.PI;
        }
        io.setSteerAngle(angleRad);
    }

    public void setDriveSpeed(double rps) {
        Logger.recordOutput("Drive Input"+SetDriveID, -rps);
        io.setDriveSpeed(rps);
    }

    public double getDriveDistance() {
        return io.getDistance();
    }

    public Rotation2d getRotation() {
        return io.getRotation();
    }

    public SwerveModuleState getCurrentState() {
        io.getData(mainData);
        Logger.recordOutput("Drive Velocity"+SetDriveID, mainData.DriveVelocity);
        Logger.recordOutput("Steer Output"+SetSteerID, mainData.SteerPosition*(2*Math.PI));
        return new SwerveModuleState(mainData.DriveVelocity,Rotation2d.fromRadians(mainData.SteerPosition*(2*Math.PI)));
    }
}
