package frc.robot.mode;

import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.ArmTools;

import java.util.Map;

public class ArmMode extends Mode {

    @Override
    public void changeMode() {

    }

    /**
     * XButton -> moveArmToSpecifiedPosition(ターゲット座標をコントローラーで変える)
     * AButton -> moveArmToSpecifiedPosition(limelightで特定された座標へ)
     * BButton -> moveArmToSpecifiedPosition(掴んだあとアームを一定の長さ垂直にあげる)
     * NoButton -> moveArmMotor / fixArmPosition (各アームの角度をコントローラーで変える -> もっとも直感的)
     */
    @Override
    public void changeState() {
        State.Arm.rightX = ArmTools.deadZoneProcess(driveController.getRightX());
        State.Arm.leftY = ArmTools.deadZoneProcess(driveController.getLeftY());

        // Xボタンが押されたら一旦Integralをリセット Targetを現在のアームの座標にリセットする
        if (driveController.getXButtonPressed()) {
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth;
            State.Arm.resetArmPidController = true;
        }

        // Aボタンが押されたら一旦Integralをリセット
        if (driveController.getAButtonPressed()) {
            State.Arm.resetArmPidController = true;
        }

        // Bボタンが押されたら一旦Integralをリセット アームを持ち上げる(Z座標を変える)
        if (driveController.getBButtonPressed()) {
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth - Const.Arm.TakeUpLengthAfterGrab;
            State.Arm.resetArmPidController = true;
        }

        if (driveController.getXButton()) {
            // Targetの座標をコントローラーによって変える　(PIDで移動する)
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.Arm.targetHeight + State.Arm.leftY * Const.Arm.TargetModifyRatio, State.Arm.targetDepth + State.Arm.rightX * Const.Arm.TargetModifyRatio)){
                State.Arm.targetHeight += State.Arm.leftY * Const.Arm.TargetModifyRatio;
                State.Arm.targetDepth += State.Arm.rightX * Const.Arm.TargetModifyRatio;
            }
        } else if (driveController.getAButton()) {
            // limelightの予測座標にターゲットを設定する　(PIDで移動する)
            // TODO ここでlimelightの値を代入
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.Arm.limelightTargetHeight, State.Arm.limelightTargetDepth)) {
                State.Arm.targetHeight = State.Arm.limelightTargetHeight;
                State.Arm.targetDepth = State.Arm.limelightTargetDepth;
            }
        } else if (driveController.getBButton()) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
        } else if (Math.abs(State.Arm.leftY) < 0.1 && Math.abs(State.Arm.rightX) < 0.1) {
            State.Arm.state = State.Arm.States.s_fixArmPosition;
        } else {
            State.Arm.state = State.Arm.States.s_moveArmMotor;
        }

        if(driveController.getLeftBumperPressed()) {
            State.Arm.resetArmEncoder = true;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = ArmTools.calculateAngles(State.Arm.targetHeight, State.Arm.targetDepth);
        State.Arm.targetRootAngle = targetAngles.get("RootAngle");
        State.Arm.targetJointAngle = targetAngles.get("JointAngle");
    }

    /**
     * @param Height : ターゲットのX座標[cm]
     * @param Depth : ターゲットのZ座標[cm]
     * この関数に座標の値域を記述する
     * @return 入力の座標が正しいか[boolean]
     */
    private boolean isNewTargetPositionInLimit(double Height, double Depth) {
        double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

        boolean isZAxisInLimit = Depth > 0;
        boolean isXAxisInLimit = Height > 0;
        boolean isInOuterBorder = length < Const.Arm.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arm.TargetPositionInnerLimit;

        return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
    }
}
