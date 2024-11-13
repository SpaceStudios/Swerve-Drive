// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain_Swerve.Drivetrain;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    mainDrive.setDefaultCommand(
      mainDrive.driveCommand(
        () -> modifyJoystick(-controller.getLeftY()), () -> modifyJoystick(-controller.getLeftX()), () -> modifyJoystick(controller.getRightX()))
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  CommandXboxController controller = new CommandXboxController(0);
  Drivetrain mainDrive = new Drivetrain(new Translation2d(0.1,0.1),new Translation2d(-0.1,0.1),new Translation2d(0.1,-0.1),new Translation2d(-0.1,-0.1));

  private double modifyJoystick(double in) {
    if (Math.abs(in) < 0.075) {
      return 0;
    }
    return in * in * Math.signum(in);
  }
}
