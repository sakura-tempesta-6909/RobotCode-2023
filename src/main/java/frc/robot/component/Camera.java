package frc.robot.component;

import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Util;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        State.cameraCenterWidth = Util.getConsole("CenterX", 0.0);
        State.cameraCenterHeight = Util.getConsole("CenterY", 0.0);
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan(State.cameraCenterWidth  / Const.Calculation.Camera.FocalLengthWeight));
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan(State.cameraCenterHeight / Const.Calculation.Camera.FocalLengthHeight));

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.aprilTagAngleHeight;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        State.cameraToTagCenterDepth = (Const.Calculation.Camera.GoalHeight - Const.Calculation.Camera.CameraLensHeight) / Math.tan(angleToGoalRadians);
        State.armRootToTagCenterDepth = State.cameraToTagCenterDepth - Const.Calculation.Camera.CameraToArmRootDepth;

    }
    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void readSensors() {
        calculation();

        // ArmStateの中に代入
        State.Arm.topCubeGoalDepth = State.armRootToTagCenterDepth + Const.Calculation.Camera.TagCenterToTopGoalDepth;
        State.Arm.middleCubeGoalDepth = State.armRootToTagCenterDepth + Const.Calculation.Camera.TagCenterToMiddleGoalDepth;
        State.Arm.bottomCubeGoalDepth = State.armRootToTagCenterDepth + Const.Calculation.Camera.TagCenterToBottomGoalDepth;
    }


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
