// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverStickPort = 1;
   //Drive motor CAN id's
    public static final int FL_Drive_Id = 1;
    public static final int FL_Steer_Id = 11;
    public static final int FR_Drive_Id = 2;
    public static final int FR_Steer_Id = 12;
    public static final int BR_Drive_Id = 3;
    public static final int BR_Steer_Id = 13;
    public static final int BL_Drive_Id = 4;
    public static final int BL_Steer_Id = 14;

  }
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 1;
    public static final double kTurningMotorGearRatio = 1 / 1;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
}

public static final double kTrackWidth = Units.inchesToMeters(22);
// Distance between right and left wheels
public static final double kWheelBase = Units.inchesToMeters(24);
// Distance between front and back wheels
public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    //     new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
}
