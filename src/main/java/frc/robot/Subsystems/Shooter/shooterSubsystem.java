// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotIDs;
import frc.robot.Constants.robotConstants;

public class shooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */
  shooterIO io;
  public shooterSubsystem() {
    switch (robotConstants.currentMode) {
      case REAL:
        io = new shooterIO_SIM(RobotIDs.ShooterID, 1);
        break;
      case SIM:
        io = new shooterIO_SIM(1, 1);
        break;
      case REPLAY:
        io = new shooterIO_SIM(1, 1);
        break;
    }
  }

  public void setShooterVolts(double volts) {
    io.setShooterVolts(volts);
  }

  public void setLauncherVolts(double volts) {
    io.setLauncherVolts(volts);
  }

  public double getShooterSpeed() {
    return io.getShooterSpeed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.motorUpdate();
    Logger.recordOutput("Shooter Speed", getShooterSpeed());
  }
}
