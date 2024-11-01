// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends Command {
  /** Creates a new Drive. */
  double inputX;
  double inputY;
  public Drive(double Axis_InputX, double Axis_InputY) {
    // Use addRequirements() here to declare subsystem dependencies.
    inputX = Axis_InputX;
    inputY = Axis_InputY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
