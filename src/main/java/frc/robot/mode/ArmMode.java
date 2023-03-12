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
        if (driveController.getStartButton()) {
            State.mode = State.Modes.k_drive;
        }
    }

    /**
     * // TODO Javadocを書く
     */
    @Override
    public void changeState() {

        final double joystickX = Tools.deadZoneProcess(joystick.getX());
        final double joystickY = Tools.deadZoneProcess(joystick.getY());
        final double joystickZ = Tools.deadZoneProcess(joystick.getZ());
        final boolean isJoystickPoleMove = (joystickX > 0) || (joystickY > 0) || (joystickZ > 0);

        if (getSeveralRawButtonPressed(new int[]{2, 6, 7, 8, 9, 10, 11, 12}) || getSeveralRawButtonReleased(new int[]{2, 6, 7, 8, 9, 10, 11, 12})) {
            State.Arm.resetPidController = true;
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth;
        }

        if (joystick.getRawButton(5)) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
        } else if (joystickZ > 0.5) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveRightMotor;
        } else if (joystickZ < -0.5) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveLeftMotor;
        } else {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        }

        //ボタン1でハンドを開く
        if (joystick.getRawButton(1)) {
            State.Hand.grabHandState = GrabHandState.s_releaseHand;
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

        State.Arm.state = State.Arm.States.s_fixArmPosition;

        if (isJoystickPoleMove && isNewTargetPositionInLimit(State.Arm.targetHeight + joystickZ * Const.Arm.TargetModifyRatio, State.Arm.targetDepth + joystickY * Const.Arm.TargetModifyRatio)) {
            //Y方向にスティックを倒してアームを前後に動かす, Z方向にスティックを曲げてアームを上下に動かす
            State.Arm.targetHeight += joystickX * Const.Arm.TargetModifyRatio;
            State.Arm.targetDepth += joystickY * Const.Arm.TargetModifyRatio;
        }

        if (joystick.getRawButton(6)) {
            // 各アームの角度をコントローラーで変える -> もっとも直感的
            State.Arm.state = State.Arm.States.s_moveArmMotor;
            State.Arm.rootSpeed = joystickX;
            State.Arm.jointSpeed = joystickY;
        }

        if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Limelight.BackGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightBack;
        }

        if (joystick.getRawButton(8)) {
            // 奥のキューブのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Camera.BackGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraBack;
        }

        if (joystick.getRawButton(9)) {
            // 真ん中のコーンのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Limelight.MiddleGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightMiddle;
        }

        if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Camera.MiddleGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraMiddle;
        }

        if (joystick.getRawButton(11)) {
            // 前のコーンのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Limelight.FrontGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightFront;
        }

        if (joystick.getRawButton(12)) {
            // 前のキューブのゴールまでアームを伸ばす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Calculation.Camera.FrontGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraFront;
        }

        if (joystick.getRawButton(2)) {
            // BasicPositionに戻る
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
        }

//        //Xの正の方向にスティックを倒してアームを右に動かす, Xの負の方向にスティックを倒してアームを左に動かす,
//        //RightBumperLeftBumper同時押しでアームの位置をリセット
//        if (driveController.getRightBumper() && driveController.getLeftBumper()) {
//            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
//        } else if (joystick.getX() > 0.5) {
//            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveRightMotor;
//        } else if (joystick.getX() > -0.5) {
//            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveLeftMotor;
//        } else {
//            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
//        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = Tools.calculateAngles(State.Arm.targetDepth, State.Arm.targetHeight);
        State.Arm.targetRootAngle = targetAngles.get("RootAngle");
        State.Arm.targetJointAngle = targetAngles.get("JointAngle");
    }

    /**
     * @param Height : ターゲットのX座標[cm]
     * @param Depth  : ターゲットのZ座標[cm]
     *               この関数に座標の値域を記述する
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

    private boolean getSeveralRawButtonPressed(int[] buttonIdxs) {
        boolean flag = false;
        for (int buttonIdx : buttonIdxs) {
            flag |= joystick.getRawButtonPressed(buttonIdx);
        }
        return flag;
    }

    private boolean getSeveralRawButtonReleased(int[] buttonIdxs) {
        boolean flag = false;
        for (int buttonIdx : buttonIdxs) {
            flag |= joystick.getRawButtonReleased(buttonIdx);
        }
        return flag;
    }
}