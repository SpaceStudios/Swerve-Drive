// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain_Swerve.Drivetrain;

public class Drive extends Command {
  /** Creates a new Drive. */
  double inputX;
  double inputY;
  Drivetrain drivetrain;
  public Drive(double Axis_InputX, double Axis_InputY, Drivetrain newDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    inputX = Axis_InputX;
    inputY = Axis_InputY;
    drivetrain = newDrivetrain;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveSwerve(inputX, inputY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
