// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climbSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.climberConstants;

/** Add your docs here. */
public class climbIO_SIM implements climbIO {
    DCMotorSim ClimbMotor;
    PIDController climbPid;
    Mechanism2d climbMechanism;
    MechanismRoot2d climbRoot;
    MechanismLigament2d climbArm;

    public climbIO_SIM(int climbMotorID) {
        ClimbMotor = new DCMotorSim(DCMotor.getNEO(1), climberConstants.climbRatio, climberConstants.climbMOI);
        climbPid = new PIDController(climberConstants.climbP, climberConstants.climbI, climberConstants.climbD);
        climbMechanism = new Mechanism2d(1, 1);
        climbRoot = climbMechanism.getRoot("ClimbRoot", 0.5, 0);
        climbArm = climbRoot.append(new MechanismLigament2d("Climb Arm", 0, Math.PI/2));
    }

    @Override
    public void setClimbVolts(double volts) {
        ClimbMotor.setInputVoltage(volts);
    }

    @Override
    public double getClimbPositionMeters() {
        return ClimbMotor.getAngularPositionRad()*climberConstants.WheelRadius;
    }

    @Override
    public void setClimbPosition(double position) {
        ClimbMotor.update(0.020);
        ClimbMotor.setInputVoltage(climbPid.calculate(ClimbMotor.getAngularPositionRad(),position/climberConstants.WheelRadius));
        climbArm.setLength(ClimbMotor.getAngularPositionRad()*climberConstants.WheelRadius);
        climbArm.setAngle(Rotation2d.fromRadians(Math.PI/2));
        Logger.recordOutput("Climb Mechanism", climbMechanism);
    }
}
