// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

/** Add your docs here. */
public interface SwerveIO {
    public class SwerveData {
        double DriveOutput;
        double SteerOutout;

        double DrivePosition;
        double SteerPosition;
    }
    public abstract void setDriveVolts(double volts);
    public abstract void setSteerAngle(double angle);
    public abstract void getData(SwerveData data);
}
