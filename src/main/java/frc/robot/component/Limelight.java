package frc.robot.component;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.LimelightState;
import frc.robot.consts.LimelightConst;


public class Limelight implements Component {
    private final NetworkTableEntry txEntry, tyEntry, tvEntry, pipelineEntry;

    public Limelight() {
        LimelightState.table = NetworkTableInstance.getDefault().getTable("limelight");
        // limelightを縦向きにしたのでtxとtyは逆
        txEntry = LimelightState.table.getEntry("ty");
        tyEntry = LimelightState.table.getEntry("tx0");
        tvEntry = LimelightState.table.getEntry("tv");
        pipelineEntry = LimelightState.table.getEntry("pipeline");

        CameraServer.startAutomaticCapture();

    }

    public void autonomousInit() {

    }

    public void teleopInit() {

    }

    public void disabledInit() {

    }

    public void testInit() {

    }

    public void readSensors() {
        double vpw = Math.tan(Math.toRadians(59.6/2));
        double targetOffsetAngle_Vertical = vpw *Math.abs(tyEntry.getDouble(0.0));
       
      
        LimelightState.tx = -txEntry.getDouble(0);
        LimelightState.tv = tvEntry.getDouble(0) != 0;

        // ターゲットまでの距離
        LimelightState.limelightToFrontGoal = Math.abs(LimelightConst.GoalHeight - LimelightConst.LimelightLensHeight) /targetOffsetAngle_Vertical;
        LimelightState.armToGoal = LimelightState.limelightToFrontGoal - LimelightConst.LimelightToArm;
        LimelightState.limelightToBackGoal = LimelightState.limelightToFrontGoal + LimelightConst.FrontGoalToBackGoal;
        LimelightState.limelightToCube = Math.abs(LimelightConst.SubStationHeight + 12  - LimelightConst.LimelightLensHeight) / targetOffsetAngle_Vertical;
        LimelightState.limelightToCone = Math.abs(LimelightConst.SubStationHeight  + 16.5 - LimelightConst.LimelightLensHeight) / targetOffsetAngle_Vertical;
        LimelightState.armToCone = LimelightState.limelightToCone - LimelightConst.LimelightToArm;
        LimelightState.armToCube = LimelightState.limelightToCube - LimelightConst.LimelightToArm;

        SmartDashboard.putNumber("FrontGoal", LimelightState.armToGoal);
        SmartDashboard.putNumber("tx", LimelightState.tx);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        SmartDashboard.putNumber("BackGoal", LimelightState.limelightToBackGoal);
        SmartDashboard.putBoolean("Limelight",  tvEntry.getDouble(0) != 0);
        SmartDashboard.putNumber("cube", LimelightState.armToCube);
        SmartDashboard.putNumber("cone", LimelightState.armToCone);

        

    }

    public void applyState() {
        if (LimelightState.isLimelightOn) {
            LimelightState.table.getEntry("ledMode").setNumber(3);
        } else if (LimelightState.isLimelightFlashing){
            LimelightState.table.getEntry("ledMode").setNumber(2);
        } else {
            LimelightState.table.getEntry("ledMode").setNumber(1);
        }

        switch (LimelightState.limelightState) {
            case s_tapeDetection:
                pipelineEntry.setNumber(0);
                break;
            case s_coneDetection:
                pipelineEntry.setNumber(1);
                break;
            case s_cubeDetection:
                pipelineEntry.setNumber(2);
        }

    }

}