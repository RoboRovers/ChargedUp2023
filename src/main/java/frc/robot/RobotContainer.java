// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.PneumaticsCommand;
import frc.robot.commands.PulleyCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveModule;
//import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.OI;
import frc.robot.Constants.OIConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> autonChooser;
  
  // all controllers will be refrenced here

  // init all our controllers and motors
  private final OI driveController = new OI(OIConstants.kDriverControllerPort);
  public SwerveSubsystem s_Swerve = new SwerveSubsystem();
  public static PneumaticsSubsystem _pneumatics = new PneumaticsSubsystem();
  private final OI flightStick = new OI(OIConstants.kDriverStickPort);
  public static PulleySubsystem _pulley = new PulleySubsystem(Constants.PullyConstants.pulleyMotorNum);
  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
  public CANSparkMax steerMotor;
  public SparkMaxPIDController turningPidController;

  public RobotContainer() {



    autonChooser = new SendableChooser<>();
    //autonChooser.addOption("AutonTest", autontest);
    SmartDashboard.putData("AutonChooser", autonChooser);

//set the swerve drive as a default command for the drive command using the driveController and the flightstick
    s_Swerve.setDefaultCommand(
        new DriveCommand(
            s_Swerve,
            driveController, flightStick));


            // _pulley.setDefaultCommand(
            //     new PulleyCommand(
            //       _pulley,
            //        opController));
          
              // _pneumatics.setDefaultCommand(
              //   new PneumaticsCommand(
                  // _pneumatics, 
              //      opController));


    // Configure the button bindings
    configureButtonBindings();


  }

  //dont think this is really necessary because we are defining them in the file OI.java
  private void configureButtonBindings() {
    /* Driver Buttons */
    opController.leftTrigger().toggleOnTrue(_pneumatics.extensionOutCommand());
    opController.leftBumper().toggleOnTrue(_pneumatics.extensionRetractCommand());
    opController.rightTrigger().whileTrue(_pneumatics.intakeOpenCommand());
    opController.rightTrigger().whileFalse(_pneumatics.intakeCloseCommand());

    opController.povLeft().whileTrue(_pulley.liftIntakeCommand());
    opController.povLeft().whileFalse(_pulley.StopCommand());
    opController.povRight().whileTrue(_pulley.dropIntakeCommand());
    opController.povRight().whileFalse(_pulley.StopCommand());

    opController.button(7).whileTrue(_pneumatics.flipperExtendCommand());
    opController.button(7).whileFalse(_pneumatics.flipperCloseCommand());

    
        //full close reset
       /* opController.button(8).toggleOnTrue(
                _pulley.homeCommand().
                alongWith(_pneumatics.extensionRetractCommand(), _pneumatics.intakeCloseCommand()));
    
          
                 //auto mid shelf then reset
           opController.a().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(pulleySubsystem.homeCommand().)
                alongWith(_pneumatics.extensionRetractCommand().
                alongWith(_pneumatics.intakeCloseCommand()))))));
    
           //mid pole command
           opController.b().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                alongWith(_pneumatics.extensionRetractCommand().
                alongWith(_pneumatics.intakeCloseCommand()))))));
    
            opController.x().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                alongWith(_pneumatics.extensionRetractCommand().
                alongWith(_pneumatics.intakeCloseCommand()))))));
    
            opController.y().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                alongWith(_pneumatics.extensionRetractCommand().
                alongWith(_pneumatics.intakeCloseCommand()))))));
                */
  }

  /*  public Command getAutonCommand() {
        
    //return autonChooser.getSelected();
    turningPidController = SwerveModule.steerMotor.getPIDController();
    

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.DriveConstants.kDriveKinematics);
                

        //TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0, 0);

//these line are where the robot will go. I need to figure out how to make multiple of these and to have the code implement
//different one of my choosing each time as well as figuring out how to do commands like picking up stuff and such.

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);



        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
          Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // 4. Construct command to follow trajectory
  SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        s_Swerve::getPose,
        Constants.DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);


        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> s_Swerve.stopModules()));

                }

}


/*Auto Options Flow chart
 * 
 * START-
 * 1. leave commmunity and hold piece                            KEEP
 * 2. place cone or cube and stay
 * 3. place and leave commmunity                                 KEEP
 * 4. 3 + drive to second piece cone or cube                     KEEP
 * 5. 1-2 + drive back to deposit ce                             KEEP
 * 6. 1-3.5 + place game piece                                   KEEP
 * 7. 1-4 + drive onto charge station but don't level
 * 8. 1-4 + drive onto charge station but auto level             KEEP
 */


}

