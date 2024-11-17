// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotIDs;
import frc.robot.Constants.drivetrainConstants;
import frc.robot.Constants.robotConstants;

public class Drivetrain extends SubsystemBase {

  SwerveDriveOdometry odometry;
  SwerveDriveKinematics kinematics;
  SwerveModulePosition[] positions;
  ChassisSpeeds speeds;
  GyroIO gyroIO;

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule rearLeft;
  SwerveModule rearRight;

  /** Creates a new Drivetrain_Swerve. */
  public Drivetrain() {
    kinematics = new SwerveDriveKinematics(drivetrainConstants.moduleFLPos,drivetrainConstants.moduleFRPos,drivetrainConstants.moduleRLPos,drivetrainConstants.moduleRRPos);

    frontLeft = new SwerveModule(0, RobotIDs.DriveFLeft, RobotIDs.SteerFLeft);
    frontRight = new SwerveModule(0, RobotIDs.DriveFRight, RobotIDs.SteerFRight);
    rearLeft = new SwerveModule(0, RobotIDs.DriveRLeft, RobotIDs.SteerRLeft);
    rearRight = new SwerveModule(0, RobotIDs.DriveRRight, RobotIDs.SteerRRight);

    positions = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);

    switch (robotConstants.currentMode) {
      case SIM:
        gyroIO = new GyroIOSim(RobotIDs.GyroID);
        break;
      default:
        gyroIO = new GyroIOSim(RobotIDs.GyroID);
        break;
    }
  }

  public void driveSwerve(double Joystick1Y, double Joystick1X, double Joystick2X) {
    double turnAngle = Joystick2X*drivetrainConstants.speedRadians;
    double moveSpeedLateral = Joystick1Y*drivetrainConstants.robotSpeedRPS;
    double moveSpeedHorizontal = Joystick1X*drivetrainConstants.robotSpeedRPS;

    Logger.recordOutput("Joystick1Y", Joystick1Y);
    Logger.recordOutput("Joystick1X", Joystick1X);
    Logger.recordOutput("Joystick2X", Joystick2X);
    Logger.recordOutput("Move Speed Lateral", moveSpeedLateral);
    Logger.recordOutput("Move Speed Horizontal", moveSpeedHorizontal);
    Logger.recordOutput("Turn Speed", turnAngle);

    ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(moveSpeedLateral,moveSpeedHorizontal,turnAngle, gyroIO.getGryoAngle());
    SwerveModuleState[] setModuleStates = kinematics.toSwerveModuleStates(newSpeeds);

    Logger.recordOutput("Swerve State Speed", setModuleStates[0].speedMetersPerSecond);
    Logger.recordOutput("Swerve Turn", setModuleStates[0].angle.getRotations());

    setModuleStates[0] = SwerveModuleState.optimize(setModuleStates[0], frontLeft.getRotation());
    setModuleStates[1] = SwerveModuleState.optimize(setModuleStates[1], frontRight.getRotation());
    setModuleStates[2] = SwerveModuleState.optimize(setModuleStates[2], rearLeft.getRotation());
    setModuleStates[3] = SwerveModuleState.optimize(setModuleStates[3], rearRight.getRotation());

    Logger.recordOutput("Intended States", setModuleStates);

    frontLeft.setDriveSpeed(setModuleStates[0].speedMetersPerSecond);
    frontLeft.setSteerAngle(setModuleStates[0].angle.getRotations());

    frontRight.setDriveSpeed(setModuleStates[1].speedMetersPerSecond);
    frontRight.setSteerAngle(setModuleStates[1].angle.getRotations());

    rearLeft.setDriveSpeed(setModuleStates[2].speedMetersPerSecond);
    rearLeft.setSteerAngle(setModuleStates[2].angle.getRotations());

    rearRight.setDriveSpeed(setModuleStates[3].speedMetersPerSecond);
    rearRight.setSteerAngle(setModuleStates[3].angle.getRotations());
  }

  public Command driveCommand(DoubleSupplier Joystick1Vertical, DoubleSupplier Joystick1Horizontal, DoubleSupplier Joystick2Horizontal) {
    return new RunCommand(() -> this.driveSwerve(Joystick1Vertical.getAsDouble(), Joystick1Horizontal.getAsDouble(),Joystick2Horizontal.getAsDouble()),this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positions = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};
    
    odometry.update(new Rotation2d(), positions);

    Logger.recordOutput("Odometry", odometry.getPoseMeters());

    SwerveModuleState frontLeftState = frontLeft.getCurrentState();
    SwerveModuleState frontRightState = frontRight.getCurrentState();
    SwerveModuleState rearLeftState = rearLeft.getCurrentState();
    SwerveModuleState rearRightState = rearRight.getCurrentState();

    SwerveModuleState[] currentStates = {frontLeftState,frontRightState,rearLeftState,rearRightState};
    Logger.recordOutput("States", currentStates);
    Logger.recordOutput("Front Left State", frontLeftState);
    Logger.recordOutput("Front Right State", frontRightState);
    Logger.recordOutput("Rear Left State", rearLeftState);
    Logger.recordOutput("Rear Right State", rearRightState);
  }
}
