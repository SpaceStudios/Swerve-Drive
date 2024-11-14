// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Pivot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.Pivot;

public class PivotChange extends Command {
  /** Creates a new PivotUp. */
  Pivot pivotSubsystem;
  double pivotChange;
  public PivotChange(Pivot setPivot, double change) {
    pivotSubsystem = setPivot;
    pivotChange = change;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.setPivotAngle(pivotChange);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
