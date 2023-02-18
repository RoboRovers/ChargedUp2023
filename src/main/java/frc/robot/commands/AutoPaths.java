package frc.robot.commands;

import java.util.List;
//controller imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
//position tracker imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//trajectory imports
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//file imports
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
//path planner imports
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathConstraints;

public class AutoPaths {
    
    public SparkMaxPIDController turningPidController;
    public CANSparkMax steerMotor;
    public SwerveSubsystem s_Swerve = new SwerveSubsystem();


    public SequentialCommandGroup leftShelfClosePiece() {

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
        PIDController thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        //ProfiledPIDController thetaController = new ProfiledPIDController(
         // Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PathPlanner.loadPath("left shelf, close", 4, 3);


  // 4. Construct command to follow trajectory
  /*SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        s_Swerve::getPose,
        Constants.DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);*/

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(PathPlanner.loadPath("left shelf, close", 4, 3), null, null, null, null, null);




        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> s_Swerve.stopModules()));

                }


                //PATH PLANNER (USE)
                //PATH WEEVER (WPI)
        }


