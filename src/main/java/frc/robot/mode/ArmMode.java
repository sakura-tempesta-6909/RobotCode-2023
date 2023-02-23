package frc.robot.mode;

import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;

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
        State.rightX = Tools.deadZoneProcess(driveController.getRightX());
        State.leftY = Tools.deadZoneProcess(driveController.getLeftY());

        // Xボタンが押されたら一旦Integralをリセット Targetを現在のアームの座標にリセットする
        if (driveController.getXButtonPressed()) {
            State.armTargetHeight = State.armActualHeight;
            State.armTargetDepth = State.armActualDepth;
            State.resetArmPidController = true;
        }

        // Aボタンが押されたら一旦Integralをリセット
        if (driveController.getAButtonPressed()) {
            State.resetArmPidController = true;
        }

        // Bボタンが押されたら一旦Integralをリセット アームを持ち上げる(Z座標を変える)
        if (driveController.getBButtonPressed()) {
            State.armTargetHeight = State.armActualHeight;
            State.armTargetDepth = State.armActualDepth - Const.Arms.TakeUpLengthAfterGrab;
            State.resetArmPidController = true;
        }

        if (driveController.getXButton()) {
            // Targetの座標をコントローラーによって変える　(PIDで移動する)
            State.armState = State.ArmState.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.armTargetHeight + State.leftY * Const.Arms.TargetModifyRatio, State.armTargetDepth + State.rightX * Const.Arms.TargetModifyRatio)){
                State.armTargetHeight += State.leftY * Const.Arms.TargetModifyRatio;
                State.armTargetDepth += State.rightX * Const.Arms.TargetModifyRatio;
            }
        } else if (driveController.getAButton()) {
            // limelightの予測座標にターゲットを設定する　(PIDで移動する)
            // TODO ここでlimelightの値を代入
            State.armState = State.ArmState.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.limelightTargetHeight, State.limelightTargetDepth)) {
                State.armTargetHeight = State.limelightTargetHeight;
                State.armTargetDepth = State.limelightTargetDepth;
            }
        } else if (driveController.getBButton()) {
            State.armState = State.ArmState.s_moveArmToSpecifiedPosition;
        } else if (Math.abs(State.leftY) < 0.1 && Math.abs(State.rightX) < 0.1) {
            State.armState = State.ArmState.s_fixArmPosition;
        } else {
            State.armState = State.ArmState.s_moveArmMotor;
        }

        if(driveController.getLeftBumperPressed()) {
            State.resetArmEncoder = true;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetThetas = Tools.calculateAngles(State.armTargetHeight, State.armTargetDepth);
        State.armTargetRootAngle = targetThetas.get("RootAngle");
        State.armTargetJointAngle = targetThetas.get("JointAngle");
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
        boolean isInOuterBorder = length < Const.Arms.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arms.TargetPositionInnerLimit;

        return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
    }
}
