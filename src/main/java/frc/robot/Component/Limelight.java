package frc.robot.Component;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;


public class Limelight implements Component {

    private NetworkTable table;
    private NetworkTableEntry txEntry, tyEntry, tvEntry;

    private double tx, ty, tv;

    public Limelight(){
        table =  NetworkTableInstance.getDefault().getTable("limelight");
        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        tvEntry = table.getEntry("tv");
        
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
        // limelightから見たターゲットの角度
        double targetOffsetAngle_Vertical = -tyEntry.getDouble(0.0);  
        
        double angleToGoalDegrees = Const.Calculation.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.toRadians(180.0));
        
        // calculate distance
        // ターゲットまでの距離
        State.distanceFromLimelightToGoalCentis = (Const.Calculation.GoalHeightCentis - Const.Calculation.LImelightLensHeightCentis)/Math.tan(angleToGoalRadians);
        //System.out.println(State.distanceFromLimelightToGoalCentis);

        tx = txEntry.getDouble(0);
        ty = tyEntry.getDouble(0);
        tv = tvEntry.getDouble(0);

        //ターゲットを追いかける
        if(Math.signum(tx) > 0) {
            State.limelightTrackingZRotation = tx / -27 * 0.5 + -0.2;
            if(tx < 9 && tx > 3){
                State.limelightTrackingZRotation = -0.5;
            }
        } else if(Math.signum(tx) < 0) {
            State.limelightTrackingZRotation = tx / -27 * 0.5 + 0.2;
            if(tx > -9 && tx < -3) {
                State.limelightTrackingZRotation = 0.5;
            }
        }
        

        //シーク
        if(tv == 0.0) {
            State.limelightSeekingZRotation = 0.5;
        } else {
            State.limelightSeekingZRotation = 0.0;
        }
       
        //ターゲットに近づく
        if(Math.signum(ty) > 0) {
            State.limelightXSpeed = ty / -20.5 * 0.5 + -0.2;
        } else if (Math.signum(ty) < 0) {
            State.limelightXSpeed = ty / 20.5 * 0.5 + 0.2;
        }

        
        //System.out.println("limelight" + State.limelight.getOrDefault("tx", txEntry.getDouble(0)));
        SmartDashboard.putNumber("distance", State.distanceFromLimelightToGoalCentis);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        
    }
    public void applyState() {

    }
    
}