// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

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
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 5;
    public static final int kBackRightDriveAbsoluteEncoderPort = 8;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

/*  public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -191.15;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -3.22;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.61;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.28;
*/
    public static final double kFLDriveAbsoluteEncoderOffsetRad = 3.35;
    public static final double kBLDriveAbsoluteEncoderOffsetRad = 3.227;
    public static final double kFRDriveAbsoluteEncoderOffsetRad = 2.035;
    public static final double kBRDriveAbsoluteEncoderOffsetRad = 4.28;

    public static final double kFRDegrees = kFRDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kFLDegrees = kFLDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kBLDegrees = kBLDriveAbsoluteEncoderOffsetRad *180 / Math.PI;
    public static final double kBRDegrees = kBRDriveAbsoluteEncoderOffsetRad *180 / Math.PI;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
}
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 7 / 1;
    public static final double kTurningMotorGearRatio = 13 / 1;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.002; //test a higher value
    public static final double kEncoderCPRSteer = 1024;
    // One revolution of the relative steering encoder. (Falcon)   
    //public static final int kEncoderCPRSteer = 26214;//it actually 26214.4, do we need the .4?  
    // One revolution of the absolute steering encoder. (CANCoder)  
    //public static final int kAbsoluteEncoderCPRSteer = 4096; //idk
      
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
}
