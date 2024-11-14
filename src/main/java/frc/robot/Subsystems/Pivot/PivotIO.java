// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot;

/** Add your docs here. */
public interface PivotIO {
    public abstract void setPivotAngle(double angleRad);
    public abstract double getPivotAngle();
    public abstract void PivotUpdate();
}
