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
}
