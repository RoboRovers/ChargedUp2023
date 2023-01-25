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
    private final SwerveModule frontLeftModule = new SwerveModule(
        2, 
        3, 
        false, 
        false);

    private final SwerveModule frontRightModule = new SwerveModule(
        7, 
        6, 
        false, 
        false);
    private final SwerveModule backLeftModule = new SwerveModule(
        12,
        11,
        false,
        false);
    private final SwerveModule backRightModule = new SwerveModule(
        10,
        9,
        false,
        false);


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
        frontLeftModule.setAngle(desiStates[0]);
        frontRightModule.setAngle(desiStates[1]);
        backLeftModule.setAngle(desiStates[2]);
        backRightModule.setAngle(desiStates[3]);
    }



}