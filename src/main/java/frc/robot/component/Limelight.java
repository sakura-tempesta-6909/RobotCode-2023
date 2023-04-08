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
        tyEntry = LimelightState.table.getEntry("tx");
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
        // limelightから受け取る情報
        // limelightから見たターゲットの角度
        double targetOffsetAngle_Vertical = -(tyEntry.getDouble(0.0) + 0.38 * 27) ;
        double gamePieceAngle = tyEntry.getDouble(0.0);
        LimelightState.tx = -txEntry.getDouble(0);
        LimelightState.tv = tvEntry.getDouble(0) != 0;
        

        //計算
        double angleToGoalDegrees = LimelightConst.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians =  Math.toRadians(angleToGoalDegrees);

        double angleToGamePieceDegrees = LimelightConst.LimelightMountAngleDegrees + gamePieceAngle;
        double angleToGamePieceRadians = Math.toRadians(angleToGamePieceDegrees);
        // ターゲットまでの距離
        LimelightState.limelightToFrontGoal = (LimelightConst.GoalHeight - LimelightConst.LimelightLensHeight) / Math.tan(angleToGoalRadians);
        LimelightState.armToGoal = LimelightState.limelightToFrontGoal - LimelightConst.LimelightToArm;
        LimelightState.limelightToBackGoal = LimelightState.limelightToFrontGoal + LimelightConst.FrontGoalToBackGoal;
        LimelightState.limelightToGamePiece = (LimelightConst.SubStationHeight - LimelightConst.LimelightLensHeight) / Math.tan(angleToGamePieceRadians);
        LimelightState.armToCone = LimelightState.limelightToGamePiece - LimelightConst.LimelightToArm;
        LimelightState.armToCube = LimelightState.limelightToGamePiece - LimelightConst.LimelightToArm;

        SmartDashboard.putNumber("FrontGoal", LimelightState.armToGoal);
        SmartDashboard.putNumber("tx", LimelightState.tx);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        SmartDashboard.putNumber("BackGoal", LimelightState.limelightToBackGoal);
        SmartDashboard.putBoolean("Limelight",  tvEntry.getDouble(0) != 0);
        

    }

    public void applyState() {
        if (LimelightState.isLimelightOn) {
            LimelightState.table.getEntry("ledMode").setNumber(3);
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