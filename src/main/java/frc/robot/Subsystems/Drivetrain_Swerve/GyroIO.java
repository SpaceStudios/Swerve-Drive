// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    public abstract Rotation2d getGryoAngle();
    public abstract void setRotation(double angle);
    public abstract void Reset();
}
