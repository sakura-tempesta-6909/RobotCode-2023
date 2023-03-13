package frc.robot.mode;

import frc.robot.State;
import frc.robot.State.GrabHandState;
import frc.robot.State.MoveLeftAndRightArmState;
import frc.robot.State.Hand.RotateState;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;

import java.util.Map;

public class ArmMode extends Mode {

    @Override
    public void changeMode() {
        if (driveController.getStartButton()){
            State.mode = State.Modes.k_drive;
        }
        if (driveController.getBackButton()){
            State.mode = State.Modes.k_arm;
        }
    }

    /**
     * XButton -> moveArmToSpecifiedPosition(ターゲット座標をコントローラーで変える)
     * AButton -> moveArmToSpecifiedPosition(limelightで特定された座標へ)
     * BButton -> moveArmToSpecifiedPosition(掴んだあとアームを一定の長さ垂直にあげる)
     * NoButton -> moveArmMotor / fixArmPosition (各アームの角度をコントローラーで変える -> もっとも直感的)
     */
    @Override
    public void changeState() {

        //ボタン2でBasicPositionに戻る
        if (joystick.getRawButton(2)) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
        }

        if (joystick.getRawButton(7)) {
            // ボタン7でTOP ROW CONE NODESまでアームを伸ばす
        } else if (joystick.getRawButton(9)) {
            // ボタン9でMiddle ROW CONE NODESまでアームを伸ばす
        } else if (joystick.getRawButton(11)) {
            // ボタン11でBottom ROW CONE NODESまでアームを伸ばす
        } else if (joystick.getRawButton(8)) {
            // ボタン8でTOP ROW CUBE NODESまでアームを伸ばす
        } else if (joystick.getRawButton(10)) {
            // ボタン10でMiddle ROW CUBE NODESまでアームを伸ばす
        } else if (joystick.getRawButton(12)) {
            // ボタン12でBottom ROW CUBE NODESまでアームを伸ばす
        } else {

        }

        //ボタン4で手首が180°回転する, ボタン5で手首が右回転する, ボタン6で手首が左回転する, ボタン3で手首の位置をリセット
        if (joystick.getRawButton(4)) {
            State.rotateState = RotateState.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            State.rotateState = RotateState.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            State.rotateState = RotateState.s_leftRotateHand;
        } else if (joystick.getRawButton(3)) {
            State.rotateState = RotateState.s_moveHandToSpecifiedAngle;
            State.Hand.targetAngle = State.Hand.actualHandAngle + 180;
        } else {
            State.rotateState = RotateState.s_stopHand;
        }

        //Y方向にスティックを倒してアームを前後に動かす, Z方向にスティックを曲げてアームを上下に動かす
        final double joystickZ = Tools.deadZoneProcess(joystick.getZ());
        final double joystickY = Tools.deadZoneProcess(joystick.getY());
        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
        if (isNewTargetPositionInLimit(State.Arm.targetHeight + joystickZ * Const.Arm.TargetModifyRatio, State.Arm.targetDepth + joystickY * Const.Arm.TargetModifyRatio)) {
            State.Arm.targetHeight += joystickZ * Const.Arm.TargetModifyRatio;
            State.Arm.targetDepth += joystickY * Const.Arm.TargetModifyRatio;
        }


        //Xの正の方向にスティックを倒してアームを右に動かす, Xの負の方向にスティックを倒してアームを左に動かす,
        //RightBumperLeftBumper同時押しでアームの位置をリセット
        if (driveController.getRightBumper() && driveController.getLeftBumper()) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
        } else if (joystick.getX() > 0.5) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveRightMotor;
        } else if (joystick.getX() > -0.5) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveLeftMotor;
        } else {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        }

        //ボタン1でハンドを開く
        if (joystick.getRawButton(1)) {
            State.Hand.grabHandState = GrabHandState.s_releaseHand;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = Tools.calculateAngles(State.Arm.targetDepth, State.Arm.targetHeight);
        State.Arm.targetRootAngle = targetAngles.get("RootAngle");
        State.Arm.targetJointAngle = targetAngles.get("JointAngle");
    }

        /**
         * @param Height : ターゲットのX座標[cm]
         * @param Depth : ターゲットのZ座標[cm]
         * この関数に座標の値域を記述する
         * @return 入力の座標が正しいか[boolean]
         */
        private boolean isNewTargetPositionInLimit ( double Height, double Depth){
            double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

            boolean isZAxisInLimit = Depth > 0;
            boolean isXAxisInLimit = Height > 0;
            boolean isInOuterBorder = length < Const.Arm.TargetPositionOuterLimit;
            boolean isOutInnerBorder = length > Const.Arm.TargetPositionInnerLimit;

            return true;
            // TODO XButtonでコントロールする時のターゲット座標の制限を考える
//        return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
        }
    }