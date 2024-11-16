// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climbSubsystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.climberConstants;

/** Add your docs here. */
public class climbIO_SIM implements climbIO {
    DCMotorSim ClimbMotor;
    
    public climbIO_SIM(int climbMotorID) {
        ClimbMotor = new DCMotorSim(DCMotor.getNEO(1), climberConstants.climbRatio, climberConstants.climbMOI);
    }

    @Override
    public void setClimbVolts(double volts) {
        ClimbMotor.setInputVoltage(volts);
    }

    @Override
    public double getClimbPosition() {
        return ClimbMotor.getAngularPositionRad();
    }
    
}
