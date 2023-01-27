package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderJNI;
import com.ctre.phoenix.sensors.SensorTimeBase;
//import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
public class SwerveModule extends SubsystemBase {


    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;
    private Rotation2d lastAngle;


    private static final double RAMP_RATE = 0.5;//1.5;
    
     /*  PID coefficients
     kP = 0.1; 
     kI = 1e-4;
     kD = 1; 
     kIz = 0; 
     kFF = 0; 
     kMaxOutput = 1; 
     kMinOutput = -1;*/
    
    private RelativeEncoder driveMotorEncoder;
    private RelativeEncoder steerMotorEncoder;
   //Check to see that this value is correct
    public double encoderCountPerRotation = 42;

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
  steerMotor.set(state.angle.getDegrees());
  //steerMotor.set(absoluteEncoder.getAbsolutePosition());
  SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());


  //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

 

  /*public void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond * 0.01))
            ? lastAngle
            : desiredState.angle;

    //SteerMotorPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    CANCoder.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }*/


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


}