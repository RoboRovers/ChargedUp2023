package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.Constants;



public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFRDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBLDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBRDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackRightTurningEncoderReversed);

   
public void ResetAllEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();
}

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
       
        SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition()* 0.3515625);
        SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition()* 0.3515625);
        SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition()* 0.3515625);
        SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition()* 0.3515625);
        
    }

 
    public double frontLeftPosition() {
        return frontLeftModule.getAbsoluteEncoderDeg();
    }
    public double frontRightPosition() {
        return frontRightModule.getAbsoluteEncoderDeg();
    }
    public double backLeftPosition() {
        return backLeftModule.getAbsoluteEncoderDeg();
    }
    public double backRightPosition() {
        return backRightModule.getAbsoluteEncoderDeg();
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
   

public void faceAllFoward() {
   /*  frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDriveAbsoluteEncoderOffsetRad);
    frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDriveAbsoluteEncoderOffsetRad);
    backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDriveAbsoluteEncoderOffsetRad);
    backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDriveAbsoluteEncoderOffsetRad);
    */
    
      frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
    frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
    backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
    backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
    System.out.println("exacuted faceAll");



   /*  frontRightModule.wheelFaceForward(0);
    frontLeftModule.wheelFaceForward(0);
    backLeftModule.wheelFaceForward(0);
    backRightModule.wheelFaceForward(0);
    */
}

}
