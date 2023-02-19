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
            State.armTargetAxisX = State.armActualAxisX;
            State.armTargetAxisZ = State.armActualAxisZ;
            State.resetArmPidController = true;
        }

        // Aボタンが押されたら一旦Integralをリセット
        if (driveController.getAButtonPressed()) {
            State.resetArmPidController = true;
        }

        // Bボタンが押されたら一旦Integralをリセット アームを持ち上げる(Z座標を変える)
        if (driveController.getBButtonPressed()) {
            State.armTargetAxisX = State.armActualAxisX;
            State.armTargetAxisZ = State.armActualAxisZ - Const.Arms.TakeUpLengthAfterGrab;
            State.resetArmPidController = true;
        }

        if (driveController.getXButton()) {
            // Targetの座標をコントローラーによって変える　(PIDで移動する)
            // TODO Armがターゲットに到達したら次のターゲットを設定する方式 (必要性要検討)

//             if(State.isArmAtTarget) {
//                 State.armTargetAxisX += State.leftY * Const.Arms.TargetModifyRatio;
//                 State.armTargetAxisZ += State.rightX * Const.Arms.TargetModifyRatio;
//                 State.resetPidController = true;
//             }
            State.armState = State.ArmState.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.armTargetAxisX + State.leftY * Const.Arms.TargetModifyRatio, State.armTargetAxisZ + State.rightX * Const.Arms.TargetModifyRatio)){
                State.armTargetAxisX += State.leftY * Const.Arms.TargetModifyRatio;
                State.armTargetAxisZ += State.rightX * Const.Arms.TargetModifyRatio;
            }
        } else if (driveController.getAButton()) {
            // limelightの予測座標にターゲットを設定する　(PIDで移動する)
            // TODO ここでlimelightの値を代入
            State.armState = State.ArmState.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.limelightTargetAxisX, State.limelightTargetAxisZ)) {
                State.armTargetAxisX = State.limelightTargetAxisX;
                State.armTargetAxisZ = State.limelightTargetAxisZ;
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
        Map<String, Double> targetThetas = Tools.calculateThetas(State.armTargetAxisX, State.armTargetAxisZ);
        State.armTargetTheta1 = targetThetas.get("theta1");
        State.armTargetTheta2 = targetThetas.get("theta2");
    }

    /**
     * input X,Z : X,Z座標の値[cm]
     * この関数に座標の値域を記述する
     * @return 入力の座標が正しいか[boolean]
     */
    private boolean isNewTargetPositionInLimit(double X, double Z) {
        double length = Math.sqrt(Math.pow(X, 2) + Math.pow(Z, 2));

        boolean isZAxisInLimit = Z > 0;
        boolean isXAxisInLimit = X > 0;
        boolean isInOuterBorder = length < Const.Arms.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arms.TargetPositionInnerLimit;

        return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
    }
}
