// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.shooterSubsystem;

public class runLauncher extends Command {
  /** Creates a new runLauncher. */
  shooterSubsystem mainSubsystem;
  double currentVolts;
  public runLauncher(shooterSubsystem currentSubsystem, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    mainSubsystem = currentSubsystem;
    currentVolts = volts;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mainSubsystem.setLauncherVolts(currentVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mainSubsystem.setLauncherVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
