// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Subsystems.Drivetrain_Swerve.SwerveIO.SwerveData;

/** Add your docs here. */
public class SwerveModule {
    SwerveModuleState currentState;
    double CurrentDistance;
    SwerveIO io;
    SwerveData mainData;
    SwerveDriveKinematics kinematics;

    public SwerveModule(SwerveModuleState newState, double Distance, int DriveID, int SteerID) {
        currentState = newState;
        CurrentDistance = Distance;
        io = new SwerveIO_SIM(DriveID, SteerID);
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition newPos = new SwerveModulePosition(CurrentDistance,currentState.angle);
        return newPos;
    }

    public void updateInputs() {
        io.getData(mainData);
        currentState = new SwerveModuleState();
    }
}
