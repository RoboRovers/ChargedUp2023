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

    opController.povUp().whileTrue(_pulley.liftIntakeCommand());
    opController.povUp().whileFalse(_pulley.StopCommand());
    opController.povDown().whileTrue(_pulley.dropIntakeCommand());
    opController.povDown().whileFalse(_pulley.StopCommand());

    opController.button(8).whileTrue(_pneumatics.flipperExtendCommand());
    opController.button(8).whileFalse(_pneumatics.flipperCloseCommand());

    
        //full close reset
        //opController.button(7).onTrue(_pneumatics.extensionRetractCommand().andThen(_pneumatics.flipperCloseCommand()).andThen(_pulley.homeCommand()));
               
    /* 
          
                 //auto mid shelf then reset
           opController.a().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(pulleySubsystem.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
           //mid pole command
           opController.b().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
            opController.x().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
            opController.y().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
                */
  }

  //  public Command getAutonCommand() {
 //} 
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




