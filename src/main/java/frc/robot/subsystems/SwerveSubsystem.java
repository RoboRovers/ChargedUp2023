package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackRightTurningEncoderReversed);


//gyro int and heading code
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left AE Value", frontLeftPosition());
        SmartDashboard.putNumber("Front Right AE Value", frontRightPosition());
        SmartDashboard.putNumber("Back Left AE Value", backLeftPosition());
        SmartDashboard.putNumber("Back Right AE Value", backRightPosition());
    }

 
    public double frontLeftPosition() {
        return frontLeftModule.absoluteEncoder.getAbsolutePosition();
    }
    public double frontRightPosition() {
        return frontRightModule.absoluteEncoder.getAbsolutePosition();
    }
    public double backLeftPosition() {
        return backLeftModule.absoluteEncoder.getAbsolutePosition();
    }
    public double backRightPosition() {
        return backRightModule.absoluteEncoder.getAbsolutePosition();
    }

    //module stops

    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
         frontLeftModule.setDesiredState(desiStates[0]);
        frontRightModule.setDesiredState(desiStates[1]);
        backLeftModule.setDesiredState(desiStates[2]);
        backRightModule.setDesiredState(desiStates[3]);
    }

   /*  public void setModuleStates(SwerveModuleState[] controllerStates) {


    }*/
    



}