// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain_Swerve.Drivetrain;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Pivot.Commands.PivotChange;
import frc.robot.Subsystems.Shooter.shooterSubsystem;
import frc.robot.Subsystems.Shooter.Commands.runLauncher;
import frc.robot.Subsystems.Shooter.Commands.runShooter;
import frc.robot.Subsystems.climbSubsystem.climbSubsystem;
import frc.robot.Subsystems.climbSubsystem.Commands.deployClimbArm;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Drivetrain mainDrive = new Drivetrain();
  Pivot mainPivot = new Pivot();
  shooterSubsystem mainShooter = new shooterSubsystem();
  climbSubsystem mainClimbSubsystem = new climbSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mainDrive.setDefaultCommand(
      mainDrive.driveCommand(
        () -> controller.getLeftY(), () -> controller.getLeftX(), () -> controller.getRightX())
    );
    controller.y().onTrue(new PivotChange(mainPivot, 1));
    controller.x().onTrue(new PivotChange(mainPivot, -1));
    controller.rightTrigger().whileTrue(new runLauncher(mainShooter, 1));
    controller.b().whileTrue(new runShooter(mainShooter, 1));
    controller.a().whileTrue(new runLauncher(mainShooter, -1)).whileTrue(new runShooter(mainShooter, -1));
    controller.leftTrigger().onTrue(new deployClimbArm(mainClimbSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}