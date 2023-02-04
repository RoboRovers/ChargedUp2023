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
  //private double absoluteEncoderOffsetRad;
//private OI driveController;



  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
  double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {
      //Create and configure a new Drive motor
      driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
      driveMotor.restoreFactoryDefaults();
      driveMotor.setInverted(invertDrive);
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
      driveMotor.setIdleMode(IdleMode.kCoast);
      
  


    //Create and configure a new steer motor
    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);
    steerMotor.setOpenLoopRampRate(RAMP_RATE);
    steerMotor.setIdleMode(IdleMode.kBrake);
    turningPidController = steerMotor.getPIDController();

    turningPidController.setP(Constants.ModuleConstants.kPTurning);


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
    steerMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderRot2Rad);
    steerMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2RadPerSec);

 //reset encoders after init phase
    resetEncoders();
    System.out.println("reset encoders");
  }


  
  
   //Motor calls
  //Get the Drive values. Value is in ticks.
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }
    //Get the Steer values. Value is in ticks.
  public double getSteerPosition() {
     return steerMotorEncoder.getPosition() * 0.3515625;
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



  //Get the absolute encoder values
    public double getAbsoluteEncoderDeg() {
    double angle = absoluteEncoder.getAbsolutePosition();
    angle *= 180 / Math.PI;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }
  
  //configure our Absolute Encoder for the MK4 drive system
  CANCoderConfiguration config = new CANCoderConfiguration();
//public SwerveModulePosition[] getstee;


//Reset encoder method. Called after init
  public void resetEncoders()  {
    driveMotorEncoder.setPosition(0);
   steerMotorEncoder.setPosition(0);
   //this might need to be on getAbsoluteEncoderDeg() not sure
   //set it to 0 to see if my keybind works
   }

  
//Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d());
  }

//This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
//location
public void setDesiredState(SwerveModuleState state) {
  //stick deadzone. No movement will occur if the stick has a value less than 0.01  
  if (Math.abs(state.speedMetersPerSecond) < 0.01) {
        stop();
        return;
  }
  
//call our drive motor and steer motor. Steer motor is multiplied by 3 to get 90deg instead of 30deg when strafing direct right/left
 driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
turningPidController.setReference(state.angle.getDegrees(), ControlType.kPosition);

//printing out or drive and steer values for debugging
  //SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
 // SmartDashboard.putNumber("Drive Speed" + driveMotor.getDeviceId(), state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  SmartDashboard.putNumber("what we are giving it" + steerMotor.getDeviceId(), state.angle.getDegrees());
  //SmartDashboard.putNumber("Cos value" +steerMotor.getDeviceId(), state.angle.getCos());
  //SmartDashboard.putNumber("Sin Value"+steerMotor.getDeviceId(), state.angle.getSin());
  //SmartDashboard.putNumber("Tan Value"+steerMotor.getDeviceId(), state.angle.getTan());
}
  
//stop method that stops the motors when the stick/s are within the deadzone < 0.01
  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }


// TO DO: if you invert this you could possibly just add the curranngle and the FFOffset and have the racks on the outside instead of 
//the inside. Try this tomorrow

//Auto face forward alg. Takes the current angle and the desired angle, foward, and sets the RE = to the offset needed to set the
//motors to facefoward. Enabling and disabling zeros the RE values. 
//YOU MUST 0 THE VALUES BEFORE DRIVING OR THE MOTORS WILL NOT FACE THE RIGHT DIRECTION 
//I should have made a button that zeros them instead of turning the bot off and on multiple times (TEST THIS)

public void wheelFaceForward(double faceForwardOffset) {
 double currangle = getAbsoluteEncoderDeg();
 double theta = (360 - (currangle - faceForwardOffset)) % 360;
 double thetaTicks = (theta/360)*Constants.ModuleConstants.kEncoderCPRSteer; 
turningPidController.setReference(thetaTicks, ControlType.kPosition);
steerMotorEncoder.setPosition(0);
}    
 }
//end of the module.
//This module is duplicated 4 times to create 4 swerve modules. Each one runs the same but does different movements based on the inputs
//given by the operator