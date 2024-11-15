// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class shooterIO_SIM implements shooterIO {
    DCMotorSim shooterMotor;
    DCMotorSim launcherMotor;

    public shooterIO_SIM(int ShooterID, int LauncherID) {
        shooterMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
        launcherMotor = new DCMotorSim(DCMotor.getNEO(1), ShooterID, LauncherID)
    }

    @Override
    public void setShooterVolts(double volts) {
        
    }

    @Override
    public void setLauncherVolts(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setLauncherVolts'");
    }

    @Override
    public double getShooterSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getShooterSpeed'");
    }
}
