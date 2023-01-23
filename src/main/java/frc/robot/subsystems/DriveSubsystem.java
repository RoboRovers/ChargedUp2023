package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private SwerveModule topLeftSwerve;
    private SwerveModule topRightSwerve;
    private SwerveModule bottomLeftSwerve;
    private SwerveModule bottomRightSwerve;

    public DriveSubsystem() {
        topLeftSwerve = new SwerveModule(2, 3);
        topRightSwerve = new SwerveModule(7, 6);
        bottomLeftSwerve = new SwerveModule(12, 11);
        bottomRightSwerve = new SwerveModule(10, 9);
    }

    public void drive(double foward, double strafe, double omega) {
        
    }

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  


    //All swerve modules brought together
    //need to be classifed in another file (ie. Swerve Module.java)
      private static SwerveModule frontLeft;
      private static SwerveModule backLeft;
      private static SwerveModule frontRight;
      private static SwerveModule backRight;

}
