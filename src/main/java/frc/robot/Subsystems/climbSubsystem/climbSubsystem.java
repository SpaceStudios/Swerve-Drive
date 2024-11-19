// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climbSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climbSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  climbIO ClimbIO;
  double setClimbPosition;
  public climbSubsystem() {
    ClimbIO = new climbIO_SIM(1);
    setClimbPosition = 0;
  }

  public void setClimbPositionMeters(double newPosition) {
    setClimbPosition = newPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("ClimbPosition", ClimbIO.getClimbPositionMeters());
    ClimbIO.setClimbPosition(setClimbPosition);
  }
}
