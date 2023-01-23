package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import javax.naming.LimitExceededException;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class SwerveModule extends SubsystemBase {


    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;

    private static final double RAMP_RATE = 0.5;//1.5;
    
    private SparkMaxPIDController SteerMotorPID;
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

    private AnalogInput absoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;

    private boolean _driveCorrect;

 

    
 

  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
   double absoluteEncoderOffsetRad, double absoluteEncoderReversed) {
    
      //Create and configure a new Drive motor
      driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
      driveMotor.restoreFactoryDefaults();
      driveMotor.setInverted(invertDrive);
      driveMotor.setOpenLoopRampRate(RAMP_RATE);
      driveMotor.setIdleMode(IdleMode.kCoast); //changed to break at comp
      driveMotor.setSmartCurrentLimit(55);


    //Create and configure a new steer motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invertSteer);
    driveMotor.setOpenLoopRampRate(RAMP_RATE);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(55);
    SteerMotorPID = steerMotor.getPIDController();
    SteerMotorPID.setP(0);
    SteerMotorPID.setI(0);
    SteerMotorPID.setD(0);
    SteerMotorPID.setFF(0);
    SteerMotorPID.setIZone(0);
    

    
  /* figure out what this does
    SteerMotorPID.setOutputRange(steerNum, driveNum);*/

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
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1 : 1);
  }
public void resetEncoders()  {
   driveMotorEncoder.setPosition(0);
  steerMotorEncoder.setPosition(getAbsoluteEncoderRad());

  }
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d()
}




  
 
 
  public void setSwerve(double angle, double speed, boolean driveCorrect) {

  this._driveCorrect = driveCorrect;
  
  //double currentAngle = getAnalogIn() % 360.0; // Use for RoboRio PID
  //double currentSteerPosition = getSteerMotorEncoder();
  //double currentAngle = currentSteerPosition % 360.0;
  //double currentAngle = getSteerMotorEncoder();
  //double targetAngle = angle; //-angle;
  //double deltaDegrees = targetAngle - currentAngle;
  double currentPosition = steerMotor.getEncoder().getPosition();
  double currentAngle = (currentPosition * 360.0 / this.encoderCountPerRotation) % 360.0;
  double targetAngle = -angle;
  double deltaDegrees = targetAngle - currentAngle;
  // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
 

  if (Math.abs(deltaDegrees) > 180.0) {
    deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
  }

  // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and only rotate by the complement
  //if (Math.abs(speed) <= MAX_SPEED){

    if (!this._driveCorrect){
      if (Math.abs(deltaDegrees) > 90.0) {
        deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
        speed = -speed;
      }
    }
  //}
  //Add change in position to current position
  //double targetPosition = currentAngle + deltaDegrees; 
  double targetPosition = currentPosition + ((deltaDegrees/360) * encoderCountPerRotation);
  //Scale the new position to match the motor encoder
  //double scaledPosition = (targetPosition / (360/STEER_MOTOR_RATIO)); 



  //check to see it "target position" is what we want to get the number of and set the angle to
  SteerMotorPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  
    }

    public void teleopPeriodic() {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    }
}