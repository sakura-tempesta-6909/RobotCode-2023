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
        // limelightから見たターゲットの角度
        double targetOffsetAngle_Vertical = -tyEntry.getDouble(0.0);

        double angleToGoalDegrees = Const.Calculation.Limelight.LimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.toRadians(180.0));
        // calculate distance
        // ターゲットまでの距離
        State.distanceFromLimelightToGoalInCM = (Const.Calculation.Limelight.GoalHeightInCM - Const.Calculation.Limelight.LimelightLensHeightInCM) / Math.tan(angleToGoalRadians);

        double tx = txEntry.getDouble(0);
        double ty = tyEntry.getDouble(0);
        double tv = tvEntry.getDouble(0);

        //ターゲットを追いかける
        if (Math.signum(tx) > 0) {
            State.limelightTrackingZRotation = tx / -Const.Calculation.Limelight.LimelightMaxHeight * Const.Speeds.MidDrive + -0.2;
            if (tx < 9 && tx > 3) {
                State.limelightTrackingZRotation = -Const.Speeds.MidDrive;
            }
        } else if (Math.signum(tx) < 0) {
            State.limelightTrackingZRotation = tx / -Const.Calculation.Limelight.LimelightMaxHeight * Const.Speeds.MidDrive + 0.2;
            if (tx > -9 && tx < -3) {
                State.limelightTrackingZRotation = Const.Speeds.MidDrive;
            }
        }

        //シーク
        if (tv == 0.0) {
            State.limelightSeekingZRotation = 0.5;
        } else {
            State.limelightSeekingZRotation = 0.0;
        }

        //ターゲットに近づく
        if (Math.signum(ty) > 0) {
            State.limelightXSpeed = ty / -Const.Calculation.Limelight.LimelightMaxWidth * Const.Speeds.MidDrive + -0.2;
        } else if (Math.signum(ty) < 0) {
            State.limelightXSpeed = ty / Const.Calculation.Limelight.LimelightMaxWidth * Const.Speeds.MidDrive + 0.2;
        }

        SmartDashboard.putNumber("distance", State.distanceFromLimelightToGoalInCM);
        SmartDashboard.putNumber("ty", tyEntry.getDouble(0));

    }

    public void applyState() {

    }

}