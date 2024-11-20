// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int theAnswerToTheGreatQuestion = 45; // It is important that you change this to a number other than 45 or the program will not run. 
    // IDs
    public class RobotIDs {
        public static final int DriveFLeft = 1;
        public static final int SteerFLeft = 2;
        public static final int DriveFRight = 3;
        public static final int SteerFRight = 4;
        public static final int DriveRLeft = 5;
        public static final int SteerRLeft = 6;
        public static final int DriveRRight = 7;
        public static final int SteerRRight = 8;
        public static final int GyroID = 9;
        public static final int ShooterID = 21;
    }

    public class drivetrainConstants {
        public static final double deadband = 0.1;

        // Drive Train PID Constants
        public static final double kPDrive = 0.01;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0.2;

        // Steer PID Constants
        public static final double kPSteer = 0.4;
        public static final double kISteer = 0;
        public static final double kDSteer = 0;

        // Drive Train Constants
        public static final double maxturnSpeedRadps = Math.PI;
        public static final double maxRobotSpeedMPS = 1;
        public static final double driveRatio = 150.0/7.0;
        public static final double driveMOI = 0.127;
        public static final double steerRatio = 150.0/7.0;
        public static final double steerMOI = 0.127;

        
        public static final double robotWidth = Units.inchesToMeters(10); // Meters
        public static final double robotLength = Units.inchesToMeters(10); // Meters
        public static final double wheelDiameter = Units.inchesToMeters(4); // Meters

        public static final double driveGearRatio = 5.36;
        public static final double turnGearRatio = 150.0 / 7.0;

        public static final double drivePositionConversionFactor = wheelDiameter * Math.PI / driveGearRatio;
        public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60;

        public static final double turnPositionConversionFactor = 2 * Math.PI / turnGearRatio;
        public static final double turnVelocityConversionFactor = turnPositionConversionFactor / 60;

        //Swerve Module Translations
        public static final Translation2d moduleFLPos = new Translation2d(-robotWidth / 2.0,  robotLength / 2.0);
        public static final Translation2d moduleFRPos = new Translation2d( robotWidth / 2.0,  robotLength / 2.0);
        public static final Translation2d moduleRLPos = new Translation2d(-robotWidth / 2.0, -robotLength / 2.0);
        public static final Translation2d moduleRRPos = new Translation2d( robotWidth / 2.0, -robotLength / 2.0);
    }
    
    public class climberConstants {
        public static final double climbRatio = 12.8;
        public static final double climbMOI = 0.004;
        public static final double climbP = 1;
        public static final double climbI = 0;
        public static final double climbD = 0;
        public static final double WheelRadius = 0.2;
        public static final double maxClimbHeight = 1;
    }

    public class pivotConstants {
        // Pivot PID Constants
        public static final double kPPivot = 3.5;
        public static final double kIPivot = 1;
        public static final double KDPivot = 1;
    }
    
    public class robotConstants {
        public static final double MotorVoltage = 12;

        public static robotMode currentMode = robotMode.SIM;

        public static enum robotMode {
            REAL,
            SIM,
            REPLAY
        }
    }
    public class ShooterConstants {
        public static final double ShooterLaunchSpeed = 100; // Desired Launch Speed in RPM
    }
}
