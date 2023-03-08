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

    }

    /**
     * XButton -> moveArmToSpecifiedPosition(ターゲット座標をコントローラーで変える)
     * AButton -> moveArmToSpecifiedPosition(limelightで特定された座標へ)
     * BButton -> moveArmToSpecifiedPosition(掴んだあとアームを一定の長さ垂直にあげる)
     * NoButton -> moveArmMotor / fixArmPosition (各アームの角度をコントローラーで変える -> もっとも直感的)
     */
    @Override
    public void changeState() {

        //YボタンでBasicPositionに戻る
        if (driveController.getYButton()){
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.basicPositionHeight;
            State.Arm.targetDepth = Const.Arm.basicPositionDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
        }

        //Aボタンでアームを前に伸ばす
        if (driveController.getAButton()){
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            //ここにConstの値を入れる
        }

        //Bボタンで手首が180°回転する, RTで手首が右回転する, LTで手首が左回転する, RTLT同時押しで手首の位置をリセット
        if (driveController.getRightTriggerAxis() > 0.5 && driveController.getLeftTriggerAxis() > 0.5){
            State.rotateState = RotateState.s_turnHandBack;
        }else if (driveController.getRightTriggerAxis() > 0.5){
            State.rotateState = RotateState.s_rotateHand;
        }else if (driveController.getLeftTriggerAxis() > 0.5){
            State.rotateState = RotateState.s_rotateHand;
        }else if (driveController.getBButton()){
            State.rotateState = RotateState.s_moveHandToSpecifiedAngle;
            //ここにConstの値を入れる
        }else {
            State.rotateState = RotateState.s_stopHand;
        }

        //左スティック前後でアームを前後に動かす, 右スティック前後でアームを上下に動かす
        final double rightY = Tools.deadZoneProcess(driveController.getRightY());;
        final double leftY = Tools.deadZoneProcess(driveController.getLeftY());   
        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            if(isNewTargetPositionInLimit(State.Arm.targetHeight + rightY * Const.Arm.TargetModifyRatio, State.Arm.targetDepth + leftY * Const.Arm.TargetModifyRatio)){
                State.Arm.targetHeight += rightY * Const.Arm.TargetModifyRatio;
                State.Arm.targetDepth += leftY * Const.Arm.TargetModifyRatio;
            }
        

        //RightBumperでアームを右に動かす, LeftBumperでアームを左に動かす, 
        //RightBumperLeftBumper同時押しでアームの位置をリセット
        if (driveController.getRightBumper() && driveController.getLeftBumper()){
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
        }else if (driveController.getRightBumper()){
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveRightMotor;
        }else if (driveController.getLeftBumper()){
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveLeftMotor;
        }else {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        }

        //Xボタンでハンドを開く
        if (driveController.getXButton()){
            State.grabHandState = GrabHandState.s_releaseHand;
        }
    }

        //ここまで


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

        return true;
        // TODO XButtonでコントロールする時のターゲット座標の制限を考える
//        return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
    }
}
