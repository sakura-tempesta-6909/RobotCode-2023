package frc.robot.component;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.subClass.Const;


public class Limelight implements Component {

    private NetworkTable table;
    private NetworkTableEntry txEntry, tyEntry, tvEntry;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        tvEntry = table.getEntry("tv");

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
        double tx = txEntry.getDouble(0);


        //計算
        double angleToGoalDegrees = Const.Calculation.Limelight.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.toRadians(180.0));
        // ターゲットまでの距離
        State.limelightToFrontGoal = (Const.Calculation.Limelight.GoalHeight - Const.Calculation.Limelight.LimelightLensHeight) / Math.tan(angleToGoalRadians);
        State.armToGoal = State.limelightToFrontGoal - Const.Calculation.Limelight.LimelightToArm;
        State.limelightToBackGoal = State.limelightToFrontGoal + Const.Calculation.Limelight.FrontGoalToBackGoal;


        //モーターを動かす
        //ターゲットの方を向く
        if (tx > 0) {
            State.limelightTrackingZRotation = tx / -Const.Calculation.Limelight.LimelightMaxAngleHeight * Const.Speeds.MidDrive + -0.2;
            if (tx < 9 && tx > 3) {
                State.limelightTrackingZRotation = -Const.Speeds.MidDrive;
            }
        } else if (tx < 0) {
            State.limelightTrackingZRotation = tx / -Const.Calculation.Limelight.LimelightMaxAngleHeight * Const.Speeds.MidDrive + 0.2;
            if (tx > -9 && tx < -3) {
                State.limelightTrackingZRotation = Const.Speeds.MidDrive;
            }
        }


        SmartDashboard.putNumber("FrontGoal", State.limelightToFrontGoal);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        SmartDashboard.putNumber("BackGoal", State.limelightToBackGoal);

    }

    public void applyState() {

    }

}