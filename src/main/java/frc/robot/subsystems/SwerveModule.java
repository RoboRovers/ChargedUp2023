package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderJNI;
import com.ctre.phoenix.sensors.SensorTimeBase;
//import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
public class SwerveModule extends SubsystemBase {


    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;
    private Rotation2d lastAngle;
    private final SparkMaxPIDController turningPidController;


    private static final double RAMP_RATE = 0.5;//1.5;
    
    
    private RelativeEncoder driveMotorEncoder;
    private RelativeEncoder steerMotorEncoder;
   //Check to see that this value is correct
    public double encoderCountPerRotation = 4096; //1024 

   public CANCoder absoluteEncoder;
  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffsetRad;




  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
  double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {
      //Create and configure a new Drive motor
      driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
      driveMotor.restoreFactoryDefaults();
      driveMotor.setInverted(invertDrive);
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
      driveMotor.setIdleMode(IdleMode.kCoast);
      driveMotor.setSmartCurrentLimit(55);
      


    //Create and configure a new steer motor
    steerMotor = new CANSparkMax(steerNum, MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);
    steerMotor.setOpenLoopRampRate(RAMP_RATE);
    steerMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setSmartCurrentLimit(55);
    turningPidController = steerMotor.getPIDController();

    turningPidController.setP(Constants.ModuleConstants.kPTurning);



    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
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



 
    resetEncoders();
  }

   //Motor calls
  //Get the Drive values Value is in motor revolutions.
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }
    //Get the Steer values Value is in motor revolutions.
  public double getSteerPosition() {
     return steerMotorEncoder.getPosition();
  }
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();
  }

  
  //Get the absolute encoder values
   /*  public double getAbsoluteEncoderRad() {
    //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    double angle = absoluteEncoder.getAbsolutePosition();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }*/
  
  CANCoderConfiguration config = new CANCoderConfiguration();
  private int failureCount;



  public void resetEncoders()  {
    driveMotorEncoder.setPosition(0);
   steerMotorEncoder.setPosition(0);
   }
   //configs to rads
   public double getAbsoluteEncoderRad() {
   config.sensorCoefficient = 2 * Math.PI / 4096.0;
   config.unitString = "rad";
   config.sensorTimeBase = SensorTimeBase.PerSecond;
   
  double angle = absoluteEncoder.getAbsolutePosition();
  angle *= 2.0 * Math.PI;
  angle -= absoluteEncoderOffsetRad;
  return angle * (absoluteEncoderReversed ? -1 : 1);

   }
  

public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
}


public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
        stop();
        return;
  }
  state = SwerveModuleState.optimize(state, gState().angle);
  driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  //steerMotor.set(CANCoder.calculate(getSteerPosition(), state.angle.getRadians()));
  turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
  //steerMotorEncoder.setPosition(state.angle.getRadians());

  //steerMotor.set(absoluteEncoder.getAbsolutePosition());
  SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());


  //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

 

  public void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01))
            ? lastAngle
            : desiredState.angle;

    //SteerMotorPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    turningPidController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }


  public double getPosition() {
    return CANCoderJNI.GetPosition(0);
}

  
  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

public double rotationValue(){

  return getAbsoluteEncoderRad();

}

/*public void wheelFaceForward(double faceForwardOffset) {
   turningPidController.setReference(faceForwardOffset, CANSparkMax.ControlType.kPosition);
  }*/


public void wheelFaceForward(double faceForwardOffset) {
 double currangle = absoluteEncoder.getAbsolutePosition();
double currangleDeg = currangle* 180 /Math.PI;
 double theta = (360 - (currangleDeg - faceForwardOffset)) % 360;
 double thetaTicks = (theta/360)*Constants.ModuleConstants.kEncoderCPRSteer; 
  SmartDashboard.putNumber("SwerveInitTicks"+ steerMotor.getDeviceId(), thetaTicks);
  
  SmartDashboard.putNumber("SwerveInitFailureCount" + steerMotor.getDeviceId(), 0); 
  
  REVLibError err = turningPidController.setReference(thetaTicks, ControlType.kPosition);
  int count = 0;
  while ( err != REVLibError.kOk ) 
  {
    System.out.println("Failed to zero "+steerMotor.getDeviceId()+": "+err); 
  failureCount++; 
  SmartDashboard.putNumber("SwerveInitFailureCount" + steerMotor.getDeviceId(), failureCount); 
  err = turningPidController.setReference(thetaTicks, ControlType.kPosition); 
  if ( count > 5)
  {
    break;
  }
  count++;
}
  steerMotorEncoder.setPosition(0);
      
      }

      

}