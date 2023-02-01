package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.OI;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveModule;

public class DriveCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final OI driveController;
    private final OI flightStick;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean fieldOriented=false;

    public DriveCommand(SwerveSubsystem swerveSubsystem, OI driveController, OI flightStick) {
                this.swerveSubsystem = swerveSubsystem;
                this.driveController = driveController;
                this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
                this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
                addRequirements(swerveSubsystem);
                this.flightStick = flightStick;
    }

    @Override
    public void initialize() {
        swerveSubsystem.faceAllFoward();
        //swerveSubsystem.ResetAllEncoders();
    }


    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = driveController.controller.getLeftX();
        double ySpeed = driveController.controller.getLeftY();
        double turningSpeed = driveController.controller.getRightX();
        SmartDashboard.putNumber("Left Stick X", driveController.controller.getLeftX());
        SmartDashboard.putNumber("Left Stick Y", driveController.controller.getLeftY() * -1);
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
       
        //double xSpeed = flightStick.flightStick.getX();
       // double ySpeed = flightStick.flightStick.getY();
        //double turningSpeed = flightStick.flightStick.getZ();
       // SmartDashboard.putNumber("Left Stick X", flightStick.flightStick.getX());
        //SmartDashboard.putNumber("Left Stick Y", flightStick.flightStick.getY());
        //SmartDashboard.putNumber("turningSpeed", turningSpeed);



        if(driveController.leftBumper.getAsBoolean())
        {
            swerveSubsystem.zeroHeading();
        }

 
        if(driveController.startButton.getAsBoolean())
        {
            fieldOriented = !fieldOriented;
        } 

       if(driveController.rightBumper.getAsBoolean())
       {
        
        }


        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        //could be causing a problem
        //xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        //ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        //turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.geRotation2d());
        } else {
            // Relative to robot
           // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed) ;

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