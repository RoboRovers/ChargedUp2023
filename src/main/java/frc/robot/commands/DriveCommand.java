package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
// could need this import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.OI;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final PulleySubsystem pulleySubsystem;
    private final OI driveController;
    private final CommandXboxController opController;
    // private final OI driveStick;
    // private final OI thetaStick;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=true;
     DigitalInput retractSwitch;


    public DriveCommand(SwerveSubsystem swerveSubsystem, OI driveController, CommandXboxController opController, PulleySubsystem pulleySubsystem) {
                this.swerveSubsystem = swerveSubsystem;
                this.driveController = driveController;
                this.pulleySubsystem = pulleySubsystem;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem);
                this.opController = opController;
                // this.driveStick = driveStick;
                // this.thetaStick = thetaStick;
                //   retractLimitSwitch = new DigitalInput(Constants.PullyConstants.retractSwitchstatePort);

    }

    @Override
    public void initialize() {
     swerveSubsystem.faceAllFoward();

     try{
        Thread.sleep(100);
        SwerveSubsystem.frontLeftModule.steerMotorEncoder.setPosition(0);
        SwerveSubsystem.frontRightModule.steerMotorEncoder.setPosition(0);
        SwerveSubsystem.backLeftModule.steerMotorEncoder.setPosition(0);
        SwerveSubsystem.backRightModule.steerMotorEncoder.setPosition(0);
     }catch (Exception i) {

     }
  
   
    }
   


    @Override
    public void execute() {

        // SmartDashboard.putBoolean("Retract State", retractSwitchState);

        // while(retractSwitchState = true) {
        //     opController.povUp().whileTrue(pulleySubsystem.liftIntakeCommand());
        //     opController.povUp().whileFalse(pulleySubsystem.StopCommand());
        // }
       
        //  if(retractSwitchState = false) {
        //     opController.povUp().whileTrue(pulleySubsystem.liftIntakeCommand());
        //     opController.povUp().whileFalse(pulleySubsystem.StopCommand());
        //     opController.povDown().whileTrue(pulleySubsystem.dropIntakeCommand());
        //     opController.povDown().whileFalse(pulleySubsystem.StopCommand());
        // }

        // boolean retractSwitchState = retractSwitch.get();


//Xbox joystick init and debugging code. Main drive method
        double xSpeed = driveController.controller.getLeftX()*-1;
        double ySpeed = driveController.controller.getLeftY()*-1;
        double turningSpeed = driveController.controller.getRightX()*-1;
        SmartDashboard.putNumber("Left Stick X", driveController.controller.getLeftX());
        SmartDashboard.putNumber("Left Stick Y", driveController.controller.getLeftY());
        SmartDashboard.putNumber("Right Stick X", driveController.controller.getRightX());
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
       
       
//flight stick init and debugging code. Alt drive method
       // double xSpeed = flightStick.flightStick.getX()*-1;
       // double ySpeed = flightStick.flightStick.getY()*-1;
       // double turningSpeed = flightStick.flightStick.getZ();
       // SmartDashboard.putNumber("Left Stick X", flightStick.flightStick.getX());
        //SmartDashboard.putNumber("Left Stick Y", flightStick.flightStick.getY());
        // SmartDashboard.putNumber("turningSpeed", turningSpeed);


       

//button mappings assiociated with the drive system or aspects of the drive system

//left bumper = reset heading for gyro
        if(driveController.povEast.getAsBoolean())
        {
            swerveSubsystem.zeroHeading();
        }
     

        // TO DO: make this button more reliable. When this button is held down it will turn it on and off multiple times per second.
        //So its not very reliable
        //use "toggle on true" or smg and change to a command not a boolean. Look into that

//start button = if field orriented is on or not. Middle right small button

if(driveController.povWest.getAsBoolean()){
    fieldOriented = !fieldOriented;
}


//if(opController.povLeft().onTrue(swerveSubsystem.fieldToggleCommand()) != null)

        
        
        SmartDashboard.putNumber("lights number", swerveSubsystem.lights.get());

        if(driveController.backButton.getAsBoolean())
        {
            swerveSubsystem.faceAllFoward();
            try{
               Thread.sleep(50);
               SwerveSubsystem.frontLeftModule.steerMotorEncoder.setPosition(0);
               SwerveSubsystem.frontRightModule.steerMotorEncoder.setPosition(0);
               SwerveSubsystem.backLeftModule.steerMotorEncoder.setPosition(0);
               SwerveSubsystem.backRightModule.steerMotorEncoder.setPosition(0);
            }catch (Exception i) {
       
            }
        }


        // 2. Apply deadband
         xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
         ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
         turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
 // TO DO: could be causing a problem. Check if this is a problem and ways to fix or implement this differently

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;


        // // 2. Apply deadband
         xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
         ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
         turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    ySpeed, xSpeed, turningSpeed, swerveSubsystem.geRotation2d());
                    swerveSubsystem.lights.set(0.57);

                    
        } else {
            // Relative to robot. 
            //Y and X speeds are switched her to make forward on the stick foward. Not left or right

            // TO DO: see what Turning speed is in and if it needs to be changed to radians or whatever
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
            swerveSubsystem.lights.set(0.59);

        }


        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}