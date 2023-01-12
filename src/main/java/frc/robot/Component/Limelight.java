package frc.robot.Component;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;;

public class Limelight implements Component {

    private NetworkTable table;
    private NetworkTableEntry entry;
    

    public Limelight(){
        table =  NetworkTableInstance.getDefault().getTable("limelight");
        entry = table.getEntry("ty");
        entry = table.getEntry("tx");
    }

    public void autonomousInit(){

    }
    public void teleopInit(){

    }
    public void disabledInit(){

    }
    public void testInit(){

    }
    public void readSensors(){
        // ターゲットの角度
        double targetOffsetAngle_Vertical = 45 - entry.getDouble(0.0);  
        // how many degrees back is your limelight rotated from perfectly vertical?
        // limelightの角度
        double limelightMountAngleDegrees = 36.0;

        // distance from the center of the Limelight lens to the floor
        // limelightの高さ
        double limelightLensHeightInches = 80.0;

        // distance from the target to the floor
        // ターゲットの高さ
        double goalHeightInches = 105.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        // ターゲットまでの距離
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        System.out.println(distanceFromLimelightToGoalInches);


    }
    public void applyState() {
    
    }

}