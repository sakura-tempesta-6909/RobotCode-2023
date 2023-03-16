package frc.robot.component;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.subClass.Const;


public class Limelight implements Component {

    private final NetworkTable table;
    private final NetworkTableEntry txEntry, tyEntry, tvEntry;

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


        // 計算
        double angleToGoalDegrees = Const.Calculation.Limelight.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        // double angleToGoalRadians = angleToGoalDegrees * (Math.toRadians(180.0));

        // ターゲットまでの距離
        State.limelightToMiddleGoalDepth = (Const.Calculation.Limelight.MiddleGoalHeight - Const.Calculation.Limelight.LimelightLensHeight) / Math.tan(angleToGoalRadians);
        State.limelightToTopGoalDepth = State.limelightToMiddleGoalDepth + Const.Calculation.Limelight.MiddleGoalToTopGoalDepth;

        State.armRootToMiddleGoalDepth = State.limelightToMiddleGoalDepth - Const.Calculation.Limelight.LimelightToArmRootDepth;

        // ArmStateの中に代入
        State.Arm.topCornGoalDepth = State.armRootToMiddleGoalDepth + Const.Calculation.Limelight.MiddleGoalToTopGoalDepth;
        State.Arm.middleCornGoalDepth = State.armRootToMiddleGoalDepth;
        State.Arm.bottomCornGoalDepth = State.armRootToMiddleGoalDepth + Const.Calculation.Limelight.MiddleGoalToBottomGoalDepth;

        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));

    }

    public void applyState() {

    }

}