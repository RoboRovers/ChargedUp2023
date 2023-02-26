package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase {
    
    private final SwerveSubsystem s_swerve;
    public final CommandXboxController driveController;
    public double xSpeed;
    public double ySpeed;
    public double turningSpeed;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;


    public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, CommandXboxController driveController) {
        this.s_swerve = swerveSubsystem;
        addRequirements(swerveSubsystem);
        
        this.driveController = driveController;
        driveController = new CommandXboxController(Constants.OIConstants.kDriverControllerPort);
        
        
        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    }


   


public void autoBalance(double gyroPitch) {
   double referencePitch = gyroPitch;


   while(gyroPitch > 5) 
        while(gyroPitch > 10 || gyroPitch < -10) {      
            try{Thread.sleep(20);}catch (Exception i) {}
            if(referencePitch - gyroPitch < 0) {
                ySpeed = -0.2;
            } else {
                ySpeed = 0.2;
            }

            if(gyroPitch > 20 || gyroPitch < -20) {
                if(ySpeed < 0){
                    ySpeed = -0.4;
                } else {
                    ySpeed = 0.4;
                }
            }
            
            if(gyroPitch > 30 || gyroPitch < -30) {
                if(ySpeed < 0){
                    ySpeed = -0.6;
                } else {
                    ySpeed = 0.6;
                }

                referencePitch = gyroPitch;
                try{Thread.sleep(20);}catch (Exception i) {}
            }
    }
    s_swerve.stopModules();
        //this should stop the modules when we are balanced


    // while(gyroPitch > 6 || gyroPitch < -6) {
    //     turningSpeed = 0; 
    //     xSpeed = 0;
    //     ySpeed = 0.2;       
    //     try{
    //         Thread.sleep(20);
    //      }catch (Exception i) {}
    //     if(gyroPitch > 6 || gyroPitch < -6) {
    //         ySpeed = -0.2;       
    //     } else {
    //         ySpeed = 0.2;       
    //     }
    //   }


}



  


    @Override
    public void execute() {
 
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds; {
        
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    ySpeed, xSpeed, turningSpeed, s_swerve.geRotation2d());
     }
     
     
     SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            s_swerve.setModuleStates(moduleStates);



        if(driveController.button(7).getAsBoolean()) {
            autoBalance(s_swerve.getPitch());
        }
        
    }


}
