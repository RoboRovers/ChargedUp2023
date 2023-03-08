package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class SwerveSubsystem extends SubsystemBase{
    //init all swerve modules 
public Spark lights;
public DigitalInput retractSwitch;
public final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
public PulleySubsystem pulleySubsystem;
public boolean retractSwitchState;
public boolean isItWorking;
public final Joystick driveStick;



    public static SwerveModule frontLeftModule = new SwerveModule(Constants.DriveConstants.kFrontLeftTurningMotorPort, Constants.DriveConstants.kFrontLeftDriveMotorPort, Constants.DriveConstants.kFrontLeftDriveEncoderReversed, Constants.DriveConstants.kFrontLeftTurningEncoderReversed, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kBLDegrees, Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public static SwerveModule frontRightModule = new SwerveModule(Constants.DriveConstants.kFrontRightTurningMotorPort, Constants.DriveConstants.kFrontRightDriveMotorPort, Constants.DriveConstants.kFrontRightDriveEncoderReversed, Constants.DriveConstants.kFrontRightTurningEncoderReversed, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kBRDegrees, Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public static SwerveModule backLeftModule = new SwerveModule(Constants.DriveConstants.kBackLeftTurningMotorPort, Constants.DriveConstants.kBackLeftDriveMotorPort, Constants.DriveConstants.kBackLeftDriveEncoderReversed, Constants.DriveConstants.kBackLeftTurningEncoderReversed, Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort, Constants.DriveConstants.kFLDegrees, Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    public static SwerveModule backRightModule = new SwerveModule(Constants.DriveConstants.kBackRightTurningMotorPort, Constants.DriveConstants.kBackRightDriveMotorPort, Constants.DriveConstants.kBackRightDriveEncoderReversed, Constants.DriveConstants.kBackRightTurningEncoderReversed, Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort, Constants.DriveConstants.kFRDegrees, Constants.DriveConstants.kBackRightTurningEncoderReversed);


   
    //reset all method. Used after face foward to then reference all RE values as 0 being the front.
public void ResetAllEncoders() {
    frontLeftModule.resetDrive();
    frontRightModule.resetDrive();
    backLeftModule.resetDrive();
    backRightModule.resetDrive();
}



//gyro int and heading code

// TODO see if changing to geRotation2d from gyro.getRotation2d worked
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics,
    geRotation2d(), new SwerveModulePosition[] {
 frontLeftModule.getPosition(),
 frontRightModule.getPosition(),
 backLeftModule.getPosition(),
backRightModule.getPosition()
    });
    
    
    public SwerveSubsystem(PulleySubsystem pulleySubsystem) {
        lights = new Spark(9);
        retractSwitch = new DigitalInput(9);
        this.pulleySubsystem = pulleySubsystem;
        driveStick = new Joystick(0);


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
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //used for Field Centric
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getRoll() {
        return gyro.getRoll();
    }




     public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(geRotation2d(), new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
           backRightModule.getPosition()},
            pose);
    }


 
    @Override
    public void periodic() {

        // frontLeftModule.steerMotorEncoder.setPosition(frontLeftModule.getAbsoluteEncoderDeg()-Constants.DriveConstants.kFLDegrees);
        // frontRightModule.steerMotorEncoder.setPosition(frontRightModule.getAbsoluteEncoderDeg()-Constants.DriveConstants.kFRDegrees);
        // backLeftModule.steerMotorEncoder.setPosition(backLeftModule.getAbsoluteEncoderDeg()-Constants.DriveConstants.kBLDegrees);
        // backRightModule.steerMotorEncoder.setPosition(backRightModule.getAbsoluteEncoderDeg()-Constants.DriveConstants.kBRDegrees);




    //retractSwitchState = retractSwitch.get();
     
       //retractSwitchCheck(retractSwitchState);

// SmartDashboard.putBoolean("Retract Switch State", retractSwitchState);
// SmartDashboard.putBoolean("Is the check running", isItWorking);

SmartDashboard.putNumber("Roll value", getRoll());

        
      

        odometer.update(geRotation2d(),  new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
           backRightModule.getPosition()});
            




//multiple debugging values are listed here. Names are self explanitory

        //Odometer and other gyro values
       // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putNumber("R2d", geRotation2d().getDegrees());
        //AE Degrees Reading
        SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg(4.62));
        SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg(63.45));
        SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg(12.392));
        SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg(298.2));
       //RE Degrees Reading
        SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
        SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
        SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
        SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());

    //    SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
    //    SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
    //    SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
    //    SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());

       //SmartDashboard.putNumber("kP Value" + SwerveModule.steerMotor.getDeviceId(), SwerveModule.getPIDController().getP());


    }
    

public void retractSwitchCheck(boolean switchState) {
    opController.povDown().whileTrue(pulleySubsystem.dropIntakeCommand().unless(() -> switchState));
    opController.povDown().whileTrue(pulleySubsystem.dropIntakeCommand().until(() -> switchState));
    opController.povUp().whileFalse(pulleySubsystem.StopCommand());
    opController.povDown().whileFalse(pulleySubsystem.StopCommand());
    opController.povUp().whileTrue(pulleySubsystem.liftIntakeCommand());

    opController.povDown().whileTrue(pulleySubsystem.dropIntakeCommand().until(() -> switchState));
    
    
    
    // if(switchState == true) {
    //     pulleySubsystem.StopCommand();
    // }

}

// if(switchState  == false) {
//     // opController.povUp().whileTrue(pulleySubsystem.liftIntakeCommand());
//     // opController.povUp().whileFalse(pulleySubsystem.StopCommand());
//     // opController.povDown().whileTrue(pulleySubsystem.dropIntakeCommand());
//     // opController.povDown().whileFalse(pulleySubsystem.StopCommand());
//     isItWorking = true;
// }else if(switchState  == true) {
//     pulleySubsystem.liftIntakeCommand();
//     new WaitCommand(1);
//     pulleySubsystem.StopCommand();

//     isItWorking = false;
// }

//stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
     


    //desired states calls. Takes multiple modules and gets/sets their modules individually apart of the SwerveDriveKinematics class
    public void setModuleStates(SwerveModuleState[] desiStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
         frontLeftModule.setDesiredState(desiStates[1]);
        frontRightModule.setDesiredState(desiStates[0]);
        backLeftModule.setDesiredState(desiStates[3]);
        backRightModule.setDesiredState(desiStates[2]);

    }


   //face forward method. Called once the bot is enabled
public void faceAllFoward() {
    backRightModule.wheelFaceForward(Constants.DriveConstants.kBRDegrees);
    frontLeftModule.wheelFaceForward(Constants.DriveConstants.kFLDegrees);
    frontRightModule.wheelFaceForward(Constants.DriveConstants.kFRDegrees);
   backLeftModule.wheelFaceForward(Constants.DriveConstants.kBLDegrees);
    System.out.println("exacuted faceAll");
}

 /* 
static PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
static PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
static PIDController thetaController = new PIDController(
  Constants.AutoConstants.kPThetaController, 0, 0);

public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
    
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
             xController, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             yController, // Y controller (usually the same values as X controller)
             thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );

 }*/
 

}
