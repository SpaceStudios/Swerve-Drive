// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Add your docs here. */
public class SwerveIO_SIM implements SwerveIO {
    TalonFX DriveMotor;
    TalonFX SteerMotor;

    VoltageOut DriveVoltage = new VoltageOut(0);
    VoltageOut SteerVoltage = new VoltageOut(0);

    public SwerveIO_SIM(int DriveID, int SteerID) {
        DriveMotor = new TalonFX(DriveID);
        SteerMotor = new TalonFX(SteerID);

        var DriveSimState = DriveMotor.getSimState();
        DriveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
        var SteerSimState = SteerMotor.getSimState();
        SteerSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    }

    @Override
    public void setDriveVolts(double volts) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setSteerAngle(double angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSteerAngle'");
    }

    @Override
    public void getData(SwerveData data) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getData'");
    }
}
