package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import frc.robot.LimelightHelpers;

public class LimeLight {
    
//public NetworkTableInstance tableInstance = 
//NetworkTableInstance.getDefault().getTable(LimelightHelpers.getLimelightNTTable);
    
public NetworkTableEntry limeLightNT = LimelightHelpers.getLimelightNTTableEntry("limeLightNT", "limelightEntry");





}
