// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.OI;
import frc.robot.Constants.OIConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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

  // Replace with CommandPS4Controller if needed
  private final OI driverController = new OI(OIConstants.kDriverControllerPort);
  public SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final OI flightStick = new OI(OIConstants.kDriverStickPort);
  public CANSparkMax steerMotor;
  public SparkMaxPIDController turningPidController;

  public RobotContainer() {

    autonChooser = new SendableChooser<>();
   // autonChooser.addOption("AutonTest", autontest);
    //SmartDashboard.putData("AutonChooser", autonChooser);


    s_Swerve.setDefaultCommand(
        new DriveCommand(
            s_Swerve,
            driverController, flightStick));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
  }

  public Command getAutonCommand() {
    return autonChooser.getSelected();
   /*  turningPidController = steerMotor.getPIDController();

        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

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
                SwerveSubsystem::getPose,
                Constants.DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                SwerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
                
        return new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.rese
        )
    }*/
  }
}
