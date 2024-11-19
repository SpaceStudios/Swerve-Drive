// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    }

    public class drivetrainConstants {
        // Drive Train PID Constants
        public static final double kPDrive = 2;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0;

        // Steer PID Constants
        public static final double kPSteer = 3.5;
        public static final double kISteer = 1;
        public static final double kDSteer = 1;

        // Drive Train Constants
        public static final double maxturnRadians = 2.0*Math.PI;
        public static final double robotSpeedRPS = 12;
        public static final double driveRatio = 12.8;
        public static final double driveMOI = 0.127;
        public static final double steerRatio = 12.8;
        public static final double steerMOI = 0.127;

        //Swerve Module Translations
        public static final Translation2d moduleFLPos = new Translation2d(0.1,0.1);
        public static final Translation2d moduleFRPos = new Translation2d(0.1,0.1);
        public static final Translation2d moduleRLPos = new Translation2d(0.1,0.1);
        public static final Translation2d moduleRRPos = new Translation2d(0.1,0.1);
    }
    
    public class climberConstants {
        public static final double climbRatio = 12.8;
        public static final double climbMOI = 0.004;
        public static final double climbP = 1;
        public static final double climbI = 0;
        public static final double climbD = 0;
        public static final double WheelRadius = 0.2;
        public static final double maxClimbHeight = 1000;
    }

    public class pivotConstants {
        // Pivot PID Constants
        public static final double kPPivot = 3.5;
        public static final double kIPivot = 1;
        public static final double KDPivot = 1;
    }
    
    public class robotConstants {
        public static final double MotorVoltage = 12;

        public static final Mode currentMode = Mode.SIM;

        public static enum Mode {
            REAL,
            SIM,
            REPLAY
        }
    }
    public class ShooterConstants {
        public static final double ShooterLaunchSpeed = 100; // Desired Launch Speed in RPM
    }
}
