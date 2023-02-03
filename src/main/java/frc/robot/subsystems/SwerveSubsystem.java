package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;



public class SwerveSubsystem extends SubsystemBase{
   // private final SwerveModulePosition frontLeftPosition = frontLeftModulePosition();
    //init all swerve modules 

    //private static frc.OI driveController;

    private final SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFRDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBLDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBRDriveAbsoluteEncoderOffsetRad, Constants.DriveConstants.kBackRightTurningEncoderReversed);

   
    //reset all method. Used after face foward to then reference all RE values as 0 being the front.
public void ResetAllEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();
}


//gyro int and heading code
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
   // new Rotation2d(0), frontLeftModule);
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //used to zero the gyro and used to refrence where the far end of the field is during comp.
    public void zeroHeading() {
        gyro.reset();
    }

    //used for debugging and field centric
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //used for Field Centric
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
   // public void resetOdometry(Pose2d pose) {
     //   odometer.resetPosition(pose, geRotation2d());
       // odometer.resetPosition(getHeading(), , pose);
    //}

    @Override
    public void periodic() {
//multiple debugging values are listed here. Names are self explanitory

        //Gyro heading degrees 
        SmartDashboard.putNumber("Robot Heading", getHeading());
        //AE Degrees Reading
        SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg());
        SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg());
       //RE Degrees Reading
        SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
        SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());
        SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
        SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
       //RE Ticks Readings
        SmartDashboard.putNumber("Back Left Ticks", backLeftModule.getSteerPosition() *2.844444444444444);
        SmartDashboard.putNumber("Back right Ticks", backRightModule.getSteerPosition()*2.844444444444444);
        SmartDashboard.putNumber("front Left Ticks", frontLeftModule.getSteerPosition()*2.844444444444444);
        SmartDashboard.putNumber("Front Right Ticks", frontRightModule.getSteerPosition()*2.844444444444444);

    }

//stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
/*public double frontLeftModulePosition() {
    return frontLeftModule.getSteerPosition();
}
public SwerveModulePosition frontRightModulePosition() {
    return frontRightModule.gState();
}
public SwerveModulePosition backLeftModulePosition() {
    return backLeftModule.gState();
}
public SwerveModulePosition backRightModulePosition() {
    return frontLeftModule.gState();
}

public SwerveSubsystem(SwerveModulePosition[] modulePositions) {
frontLeftModulePosition();
frontRightModulePosition();
backLeftModulePosition();
backRightModulePosition();
}*/
    //desired states calls. Takes multiple modules and gets/sets their modules individually apart of the SwerveDriveKinematics class
    public void setModuleStates(SwerveModuleState[] desiStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
         frontLeftModule.setDesiredState(desiStates[0]);
        frontRightModule.setDesiredState(desiStates[1]);
        backLeftModule.setDesiredState(desiStates[2]);
        backRightModule.setDesiredState(desiStates[3]);

    }
   //face forward method. Called once the bot is enabled
public void faceAllFoward() {
      frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
    frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
    backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
    backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
    System.out.println("exacuted faceAll");
}

}
