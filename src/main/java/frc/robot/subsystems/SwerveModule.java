package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
public class SwerveModule extends SubsystemBase {

  //initalize all variables
    public CANSparkMax steerMotor;
    public CANSparkMax driveMotor;
    public final SparkMaxPIDController turningPidController;
    private static final double RAMP_RATE = 0.5;//1.5;
    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder steerMotorEncoder;
   //Check to see that this value is correct
    public double encoderCountPerRotation = 1024; //1024 
    public CANCoder absoluteEncoder;
    private boolean absoluteEncoderReversed;



  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
  double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {
      //Create and configure a new Drive motor
      driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
      driveMotor.restoreFactoryDefaults();
      driveMotor.setInverted(invertDrive);
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
      driveMotor.setIdleMode(IdleMode.kBrake);
      
  


    //Create and configure a new steer motor
    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);
    steerMotor.setOpenLoopRampRate(RAMP_RATE);
    steerMotor.setIdleMode(IdleMode.kBrake);
    turningPidController = steerMotor.getPIDController();

    turningPidController.setP(Constants.ModuleConstants.kPTurning);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(1080); 
    turningPidController.setPositionPIDWrappingMinInput(720);



    //this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderId);
    
    //Create the built in motor encoders
 
    //Drive motor encoder
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
    driveMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    //Steer motor encoder 
    steerMotorEncoder = steerMotor.getEncoder();
    steerMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningConversionFactor2Deg);
   // Constants.ModuleConstants.kTurningEncoderRot2Deg
    steerMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2DegPerSec);

 //reset encoders after init phase
    resetDrive();
    System.out.println("reset encoders");
  }
  
//Reset encoder method. Called after init
public void resetEncoders()  {
  steerMotorEncoder.setPosition(0);
  //this might need to be on getAbsoluteEncoderDeg() not sure
  //set it to 0 to see if my keybind works
  }
  public void resetDrive() {
   driveMotorEncoder.setPosition(0);
   steerMotorEncoder.setPosition(0);

  }
//stop method that stops the motors when the stick/s are within the deadzone < 0.01
public void stop() {
  driveMotor.set(0);
  steerMotor.set(0);
}

    //configure our Absolute Encoder for the MK4 drive system
  CANCoderConfiguration config = new CANCoderConfiguration();

  //Get the absolute encoder values
  public double getAbsoluteEncoderDeg() {
    double angle = absoluteEncoder.getAbsolutePosition();
    angle *= 180 / Math.PI;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }
  
  //Motor calls
  //Get the Drive values. Value is in degrees.
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }
    //Get the Steer values. Value is in degrees.
  public double getSteerPosition() {
     return steerMotorEncoder.getPosition();
  }
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();
  }
 //Motor encoder conversions and useful info
  //2.844444444444444 * 1 = ticks. Degrees to ticks
  //1 degree equals 2.844444444444444 ticks
  //1 tick equals 0.3515625 degrees. 
  //Ticks * 0.3515625 equals degrees
  //Motors are in ticks by default 1024 ticks equals 360 deg


  
//Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d());
  }
  //it seems that all encoders values are +- 5 degrees off. this variable tries to correct that.
public double encoderCorrection;

//This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
//location
public void setDesiredState(SwerveModuleState state) {
  //stick deadzone. No movement will occur if the stick has a value less than 0.01  
  if (Math.abs(state.speedMetersPerSecond) < 0.01) {
        stop();
        return;
  }

  //this is what encoderCorrection is. it adds 5 if the encoder is positive and subtracts 5 if the value is negitive
  double encoderCorrection; {
    encoderCorrection = 0;
    if ((state.angle.getDegrees() < 5) || (state.angle.getDegrees() > -5)) {
      encoderCorrection = 0;
    }
    if (state.angle.getDegrees() > (0)) {
        encoderCorrection = 5;
      }
    if (state.angle.getDegrees() < (0)){
        encoderCorrection = -5;
       }
    }

//call our drive motor and steer motor. Steer motor is multiplied by 3 to get 90deg instead of 30deg when strafing direct right/left
 driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
turningPidController.setReference(state.angle.getDegrees()+encoderCorrection, ControlType.kPosition);
//turningPidController.setReference(-90+encoderCorrection, ControlType.kPosition);

//SmartDashboard debug stuff, printing out our drive and steer values for debugging. Uncomment as needed

/* 
//SmartDashboard.putNumber("Speed", getDriveVelocity());
  SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
 SmartDashboard.putNumber("Drive Speed" + driveMotor.getDeviceId(), state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  SmartDashboard.putNumber("what we are giving it" + steerMotor.getDeviceId(), state.angle.getDegrees());
  */
}
  



//Auto face forward alg. Takes the current angle and the desired angle, foward, and sets the RE = to the offset needed to set the
//motors to facefoward. Enabling and disabling zeros the RE values. 
//YOU MUST 0 THE VALUES BEFORE DRIVING OR THE MOTORS WILL NOT FACE THE RIGHT DIRECTION 
//I should have made a button that zeros them instead of turning the bot off and on multiple times, I need to make this automatic

public void wheelFaceForward(double faceForwardOffset) {
  double currangle = getAbsoluteEncoderDeg();
  //steerMotorEncoder.setPosition(currangle);

//repeat this
  turningPidController.setReference(faceForwardOffset, ControlType.kPosition);

//until steerMotorEncoder +- 2deg
}

}  





//end of the module.
//This module is duplicated 4 times to create 4 swerve modules. Each one runs the same but does different movements based on the inputs
//given by the operator