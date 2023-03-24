package frc.robot.component;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.LimelightState;
import frc.robot.States.State;
import frc.robot.subClass.Const;


public class Limelight implements Component {
    private final NetworkTableEntry txEntry, tyEntry, tvEntry;

    public Limelight() {
        LimelightState.table = NetworkTableInstance.getDefault().getTable("limelight");
        // limelightを縦向きにしたのでtxとtyは逆
        txEntry = LimelightState.table.getEntry("ty");
        tyEntry = LimelightState.table.getEntry("tx");
        tvEntry = LimelightState.table.getEntry("tv");

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
        double targetOffsetAngle_Vertical = -tyEntry.getDouble(0.0);
        State.tx = -txEntry.getDouble(0);
        State.tv = tvEntry.getDouble(0) != 0;
        

        //計算
        double angleToGoalDegrees = Const.Calculation.Limelight.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians =  Math.toRadians(angleToGoalDegrees);
        // ターゲットまでの距離
        State.limelightToFrontGoal = (Const.Calculation.Limelight.GoalHeight - Const.Calculation.Limelight.LimelightLensHeight) / Math.tan(angleToGoalRadians);
        State.armToGoal = State.limelightToFrontGoal - Const.Calculation.Limelight.LimelightToArm;
        State.limelightToBackGoal = State.limelightToFrontGoal + Const.Calculation.Limelight.FrontGoalToBackGoal;

        SmartDashboard.putNumber("FrontGoal", State.limelightToFrontGoal);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        SmartDashboard.putNumber("BackGoal", State.limelightToBackGoal);
        SmartDashboard.putBoolean("Limelight",  tvEntry.getDouble(0) != 0);
        SmartDashboard.putNumber("armToFrontGoal", State.armToGoal);

    }

    public void applyState() {
        if (LimelightState.isLimelightOn) {
            LimelightState.table.getEntry("ledMode").setNumber(3);
        } else {
            LimelightState.table.getEntry("ledMode").setNumber(1);
        }

    }

}