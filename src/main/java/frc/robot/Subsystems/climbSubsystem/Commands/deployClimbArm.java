// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climbSubsystem.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.climberConstants;
import frc.robot.Subsystems.climbSubsystem.climbSubsystem;

public class deployClimbArm extends Command {
  /** Creates a new deployClimbArm. */
  climbSubsystem climber;
  public deployClimbArm(climbSubsystem newclimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = newclimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setClimbPositionMeters(climberConstants.maxClimbHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimbPositionMeters(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
