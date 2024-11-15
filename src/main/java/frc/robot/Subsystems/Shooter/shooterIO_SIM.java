// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.universalConstants;

/** Add your docs here. */
public class shooterIO_SIM implements shooterIO {
    DCMotorSim shooterMotor;
    DCMotorSim launcherMotor;

    public shooterIO_SIM(int ShooterID, int LauncherID) {
        shooterMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
        launcherMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
    }

    @Override
    public void setShooterVolts(double volts) {
        shooterMotor.setInputVoltage(volts*universalConstants.MotorVoltage);
    }

    @Override
    public void setLauncherVolts(double volts) {
        
        launcherMotor.setInputVoltage(volts*universalConstants.MotorVoltage);
    }

    @Override
    public double getShooterSpeed() {
        return shooterMotor.getAngularVelocityRPM();
    }

    public void motorUpdate() {
        shooterMotor.update(0.020);
        launcherMotor.update(0.020);
    }
}
