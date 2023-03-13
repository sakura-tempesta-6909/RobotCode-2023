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
     * ** [ ]内の数字は優先順位<br><br>
     * Button 1 - ハンドを開く[0]<br>
     * Button 2 - すべてリセット　BasicPositionに戻る[0]<br><br>
     * Button 3 - 手首の位置をリセット[4]<br>
     * Button 4 - 手首を180°回転する[3]<br>
     * Button 5 - 手首が右回転する[2]<br>
     * Button 6 - 手首が左回転する[1]<br><br>
     * Button 7 - 奥のコーンのゴールまでアームを伸ばす[2]<br>
     * Button 8 - 奥のキューブのゴールまでアームを伸ばす[5]<br>
     * Button 9 - 真ん中のコーンのゴールまでアームを伸ばす[3]<br>
     * Button 10 - 真ん中のキューブのゴールまでアームを伸ばす[6]<br>
     * Button 11 - 前のコーンのゴールまでアームを伸ばす[4]<br>
     * Button 12 - 前のキューブのゴールまでアームを伸ばす[7]<br>
     * Axis1 - joystickX -> 上下移動（右に傾けると上行き）[8]<br>
     * Axis2 - joystickY -> 前後移動（前に傾けると奥行き）[8]<br>
     * Axis3 - joystickZ -> 左右移動（時計回りだと右行き）[8]<br>
     * Axis4 - 負の時MoveArmMotorに切り替え[1]の時は<br>joystickXで根本の回転 joystickYで関節部分の回転<br><br>
     * No Button - すべて停止[100]<br>
     */
    @Override
    public void changeState() {

        final double joystickX = Tools.deadZoneProcess(joystick.getRawAxis(1));
        final double joystickY = Tools.deadZoneProcess(joystick.getRawAxis(2));
        final double joystickZ = Tools.deadZoneProcess(joystick.getRawAxis(3));

        if (driveController.getRightBumper() && driveController.getLeftBumper()) {
            // アームの位置をリセット
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
        } else if (joystickZ > 0.5) {
            // アームを右に動かす
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveRightMotor;
        } else if (joystickZ < -0.5) {
            // アームを左に動かす
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_moveLeftMotor;
        }

        if (joystick.getRawButton(1)) {
            // ハンドを開く
            State.Hand.grabHandState = GrabHandState.s_releaseHand;
        }

        if (joystick.getRawButton(4)) {
            // 手首が180°回転する
            State.rotateState = RotateState.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            // 手首が右回転する
            State.rotateState = RotateState.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            // 手首が左回転する
            State.rotateState = RotateState.s_leftRotateHand;
        } else if (joystick.getRawButton(3)) {
            // 手首の位置をリセット
            State.rotateState = RotateState.s_moveHandToSpecifiedAngle;
            State.Hand.targetAngle = State.Hand.actualHandAngle + 180;
        }

        if (getSeveralRawButtonPressed(new int[]{2, 6, 7, 8, 9, 10, 11, 12}) || getSeveralRawButtonReleased(new int[]{2, 6, 7, 8, 9, 10, 11, 12})) {
            State.Arm.resetPidController = true;
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth;
        }

        if (getSeveralRawButton(new int[]{7, 8, 9, 10, 11, 12})) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
        }

        if (joystick.getRawAxis(4) < -0.8) {
            // 各アームの角度をコントローラーで変える -> もっとも直感的
            State.Arm.state = State.Arm.States.s_moveArmMotor;
            State.Arm.rootSpeed = joystickX;
            State.Arm.jointSpeed = joystickY;
        } else if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Limelight.BackGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightBack;
        } else if (joystick.getRawButton(9)) {
            // 真ん中のコーンのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Limelight.MiddleGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightMiddle;
        } else if (joystick.getRawButton(11)) {
            // 前のコーンのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Limelight.FrontGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.LimelightFront;
        } else if (joystick.getRawButton(8)) {
            // 奥のキューブのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Camera.BackGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraBack;
        } else if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Camera.MiddleGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraMiddle;
        } else if (joystick.getRawButton(12)) {
            // 前のキューブのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Camera.FrontGoalHeight - Const.Arm.RootHeight;
            State.Arm.targetDepth = State.Arm.TargetDepth.CameraFront;
        } else if ((joystickX != 0 || joystickY != 0)) {
            // X方向にスティックを曲げてアームを上下に動かす, Y方向にスティックを倒してアームを前後に動かす
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight += joystickX * Const.Arm.TargetModifyRatio;
            State.Arm.targetDepth += joystickY * Const.Arm.TargetModifyRatio;
        }

        if (joystick.getRawButton(2)) {
            // すべてBasicPositionに戻る
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
        }

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
        // return isXAxisInLimit && isZAxisInLimit && isInOuterBorder && isOutInnerBorder;
    }

    private boolean getSeveralRawButton(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButton(buttonIdx);
        }
        return flag;
    }

    private boolean getSeveralRawButtonPressed(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonPressed(buttonIdx);
        }
        return flag;
    }

    private boolean getSeveralRawButtonReleased(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonReleased(buttonIdx);
        }
        return flag;
    }
}