// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import org.opencv.calib3d.StereoBM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveIO_SIM implements SwerveIO {
    TalonFX DriveMotor;
    TalonFX SteerMotor;

    VoltageOut DriveVoltage = new VoltageOut(0);
    VoltageOut SteerVoltage = new VoltageOut(0);
    TalonFXSimState DriveSimState;
    TalonFXSimState SteerSimState;

    public SwerveIO_SIM(int DriveID, int SteerID) {
        DriveMotor = new TalonFX(DriveID);
        SteerMotor = new TalonFX(SteerID);

        DriveSimState = DriveMotor.getSimState();
        DriveSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

        // Drive PIDs
        var DrivePIDConfig = new Slot0Configs();
        DrivePIDConfig.kP = Constants.kPDrive;
        DrivePIDConfig.kI = Constants.kIDrive;
        DrivePIDConfig.kD = Constants.KDDrive;

        //Steer PIDs
        var SteerPIDConfig = new Slot0Configs();
        SteerPIDConfig.kP = Constants.kPSteer;
        SteerPIDConfig.kI = Constants.kISteer;
        SteerPIDConfig.kD = Constants.KDSteer;

        DriveMotor.getConfigurator().apply(DrivePIDConfig);
        SteerMotor.getConfigurator().apply(SteerPIDConfig);

        SteerSimState = SteerMotor.getSimState();
        SteerSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    }

    @Override
    public void setDriveVolts(double volts) {
        DriveMotor.setControl(new VelocityVoltage(volts));
    }

    @Override
    public void setSteerAngle(double angle) {
        SteerMotor.setControl(new PositionVoltage(angle/(Math.PI*2.0)));
    }

    @Override
    public void getData(SwerveData data) {
        data.DriveVelocity = DriveMotor.getVelocity().getValueAsDouble();
        data.SteerPosition = SteerMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getDistance() {
        return DriveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(SteerMotor.getPosition().getValueAsDouble());
    }
}
