// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {



    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23.75);
    // Distance between front and back wheels


public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
     
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front left
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //back right



    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 11;
    public static final int kFrontRightDriveMotorPort = 6;
    public static final int kBackRightDriveMotorPort = 9;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kBackLeftTurningMotorPort = 12;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final int kBackRightTurningMotorPort = 10;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;//broke auto
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 5;
    public static final int kBackRightDriveAbsoluteEncoderPort = 8;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;//broke auto
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

/*  public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -191.15;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -3.22;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.61;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.28;
*/

    public static final double kBRDegrees = 63.45;
    public static final double kBLDegrees = 4.62;
    public static final double kFLDegrees = 12.392;
    public static final double kFRDegrees = 298;
    
/* 
    public static final double kBRDegrees = kFRDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kBLDegrees = kFLDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kFLDegrees = kBLDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kFRDegrees = kBRDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    */
 

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4196;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;


    //4 is medium, 2.75 is pretty fast, >2 is to fast. <4 is to slow
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2.75;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 7 / 1;
    public static final double kTurningMotorGearRatio = 12.8 / 1;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    
    //get this a little closer. maybe 28.23
    public static final double kTurningConversionFactor2Deg =  28.25;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2DegPerSec = kTurningConversionFactor2Deg / 60;
    public static final double kPTurning = 0.002; //test a higher value
    public static final double kEncoderCPRSteer = 1024;
}

//Max speed
    public static final class OIConstants {
      public static final int kDriverControllerPort = 1;
      public static final int kDriverStickPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.05;
  }
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond =  DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    //public static final Trajector

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
  }


   //Pneumatics Constants
   public static final class PneumaticsConstants {
    public static final int L_INTAKE_IN = 0;
    public static final int L_INTAKE_OUT = 1;
    public static final int R_INTAKE_IN = 2;
    public static final int R_INTAKE_OUT = 5;
   //Extension number
public static final int EXTENSION_IN = 6;
public static final int EXTENSION_OUT = 7;



    //in = 0,1,2
    //out = 7,6,5




  }
}
