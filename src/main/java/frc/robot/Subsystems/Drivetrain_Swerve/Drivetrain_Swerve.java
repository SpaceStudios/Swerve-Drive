// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain_Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain_Swerve extends SubsystemBase {

  SwerveDriveOdometry odometry;
  SwerveDriveKinematics kinematics;
  SwerveModulePosition[] positions;
  ChassisSpeeds speeds;

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule rearLeft;
  SwerveModule rearRight;

  /** Creates a new Drivetrain_Swerve. */
  public Drivetrain_Swerve(Translation2d module1Pos, Translation2d module2Pos, Translation2d module3Pos, Translation2d module4Pos) {
    speeds = new ChassisSpeeds();
    positions = new SwerveModulePosition[] {new SwerveModulePosition()};
    kinematics = new SwerveDriveKinematics(module1Pos, module2Pos, module3Pos, module4Pos);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    frontLeft = new SwerveModule(moduleStates[0], 0);
    frontRight = new SwerveModule(moduleStates[1], 0);
    rearLeft = new SwerveModule(moduleStates[2], 0);
    rearRight = new SwerveModule(moduleStates[3], 0);
    positions = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
