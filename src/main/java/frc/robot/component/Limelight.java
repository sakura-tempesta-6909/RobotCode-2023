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
        State.tx = txEntry.getDouble(0);


        //計算
        double angleToGoalDegrees = Const.Calculation.Limelight.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.toRadians(180.0));
        // ターゲットまでの距離
        State.limelightToMiddleRowCornNodes = (Const.Calculation.Limelight.GoalHeight - Const.Calculation.Limelight.LimelightLensHeight) / Math.tan(angleToGoalRadians);
        State.limelightToTopRowCornNodes = State.limelightToMiddleRowCornNodes + Const.Calculation.Limelight.TopRowToMiddleRowCornNodes;
        State.limelightToBottomRowCornNodes = State.limelightToMiddleRowCornNodes - Const.Calculation.Limelight.MiddleRowToBottomRowCornNodes;
        State.armToMiddleRowCornNodes = State.limelightToMiddleRowCornNodes - Const.Calculation.Limelight.LimelightToArm;
        State.armToTopRowCornNodes = State.limelightToTopRowCornNodes - Const.Calculation.Limelight.LimelightToArm;
        State.armToBottomRowCornNodes = State.limelightToBottomRowCornNodes - Const.Calculation.Limelight.LimelightToArm;


        SmartDashboard.putNumber("FrontGoal", State.limelightToMiddleRowCornNodes);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));
        SmartDashboard.putNumber("BackGoal", State.limelightToTopRowCornNodes);

    }

    public void applyState() {

    }

}