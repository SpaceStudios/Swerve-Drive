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

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mainDrive.setDefaultCommand(
      mainDrive.driveCommand(
        () -> -modifyJoystick(controller.getLeftY()), () -> -modifyJoystick(controller.getLeftX()), () -> modifyJoystick(controller.getRightX()))
    );
    controller.y().onTrue(new PivotChange(mainPivot, 1));
    controller.x().onTrue(new PivotChange(mainPivot, -1));
    controller.rightTrigger().whileTrue(new runLauncher(mainShooter, 1));
    controller.b().whileTrue(new runShooter(mainShooter, 1));
    controller.a().whileTrue(new runLauncher(mainShooter, -1)).whileTrue(new runShooter(mainShooter, -1));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  CommandXboxController controller = new CommandXboxController(0);
  Drivetrain mainDrive = new Drivetrain();
  Pivot mainPivot = new Pivot();
  shooterSubsystem mainShooter = new shooterSubsystem();

  private double modifyJoystick(double in) {
    if (Math.abs(in) < 0.1) {
      return 0;
    }
    return in * in * Math.signum(in);
  }
}
