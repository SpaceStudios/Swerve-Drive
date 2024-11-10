// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

/** Add your docs here. */
public class GyroIOSim implements GyroIO{
    AnalogGyroSim gyroSim;

    public GyroIOSim(int GyroID) {
        gyroSim = new AnalogGyroSim(GyroID);
    }

    @Override
    public Rotation2d getGryoAngle() {
        return new Rotation2d();
    }

    @Override
    public void setRotation(double angle) {
        
    }

    @Override
    public void Reset() {
    }
}
