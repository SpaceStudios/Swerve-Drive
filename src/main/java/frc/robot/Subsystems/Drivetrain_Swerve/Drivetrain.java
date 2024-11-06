// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import java.lang.invoke.ConstantCallSite;
import java.util.Currency;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.opencv.dnn.Model;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  SwerveDriveOdometry odometry;
  SwerveDriveKinematics kinematics;
  SwerveModulePosition[] positions;
  ChassisSpeeds speeds;

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule rearLeft;
  SwerveModule rearRight;

  /** Creates a new Drivetrain_Swerve. */
  public Drivetrain(Translation2d module1Pos, Translation2d module2Pos, Translation2d module3Pos, Translation2d module4Pos) {
    speeds = new ChassisSpeeds();
    positions = new SwerveModulePosition[] {new SwerveModulePosition()};
    kinematics = new SwerveDriveKinematics(module1Pos, module2Pos, module3Pos, module4Pos);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    frontLeft = new SwerveModule(moduleStates[0], 0, Constants.DriveFLeft, Constants.SteerFLeft);
    frontRight = new SwerveModule(moduleStates[1], 0, Constants.DriveFRight, Constants.SteerFRight);
    rearLeft = new SwerveModule(moduleStates[2], 0, Constants.DriveRLeft, Constants.SteerRLeft);
    rearRight = new SwerveModule(moduleStates[3], 0, Constants.DriveRRight, Constants.SteerRRight);
    positions = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);
  }

  public void driveSwerve(double JoystickX, double JoystickY) {
    var turnAngle = JoystickX*Constants.speedRadians;
    var moveSpeed = JoystickY*Constants.robotSpeedRPS;

    frontLeft.setSteerAngle(turnAngle);
    frontLeft.setDriveVolts(moveSpeed);
    frontRight.setSteerAngle(turnAngle);
    frontRight.setSteerAngle(moveSpeed);
    rearLeft.setSteerAngle(turnAngle);
    rearLeft.setDriveVolts(moveSpeed);
    rearRight.setSteerAngle(turnAngle);
    rearRight.setDriveVolts(moveSpeed);
  }

  public Command driveCommand(DoubleSupplier left, DoubleSupplier right) {
    return new RunCommand(() -> this.driveSwerve(left.getAsDouble(), right.getAsDouble()),this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positions = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};
    odometry.update(Rotation2d.fromRadians(0), positions);
    Logger.recordOutput("Odometry", odometry.getPoseMeters());

    SwerveModuleState frontLeftState = new SwerveModuleState(frontLeft.getDriveDistance(), frontLeft.getRotation());
    SwerveModuleState frontRightState = new SwerveModuleState(frontRight.getDriveDistance(), frontRight.getRotation());
    SwerveModuleState rearLeftState = new SwerveModuleState(rearLeft.getDriveDistance(), rearLeft.getRotation());
    SwerveModuleState rearRightState = new SwerveModuleState(rearRight.getDriveDistance(), rearRight.getRotation());

    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(frontLeftState, frontRightState, rearLeftState, rearRightState);

    Logger.recordOutput("Chassis Speeds", currentSpeeds);
    SwerveModuleState[] currentStates = {frontLeftState,frontRightState,rearLeftState,rearRightState};
    Logger.recordOutput("States", currentStates);
  }
}
