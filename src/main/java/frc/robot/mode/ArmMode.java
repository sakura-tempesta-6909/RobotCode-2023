package frc.robot.mode;

import frc.robot.States.LimelightState;
import frc.robot.States.State;
import frc.robot.States.State.GrabHandState;
import frc.robot.States.State.IntakeExtensionState;
import frc.robot.States.State.MoveLeftAndRightArmState;
import frc.robot.States.State.Hand.RotateState;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmMode extends Mode {
    private static DriveMode.GrabGamePiecePhase phase = DriveMode.GrabGamePiecePhase.Phase1;

    /**
     * StartButton - DriveModeに
     */
    @Override
    public void changeMode() {
        if (driveController.getStartButton()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
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
     * Axis4 - 負（< -0.8）の時、MoveArmMotorに切り替え[1]<br>その時、joystickXで根本の回転 joystickYで関節部分の回転<br><br>
     * No Button - すべて停止[100]<br>
     */
    @Override
    public void changeState() {

        State.Drive.xSpeed = -driveController.getLeftY();
        State.Drive.zRotation = -driveController.getRightX();
        State.Drive.state = State.Drive.States.s_midDrive;

        LimelightState.isLimelightOn = true;

        if (driveController.getAButton()) {
            State.intakeExtensionState = IntakeExtensionState.s_openIntake;
        } else {
            State.intakeExtensionState = IntakeExtensionState.s_closeIntake;
        }

        final double joystickX = -1 * Tools.deadZoneProcess(joystick.getRawAxis(0));
        final double joystickY = 1 * Tools.deadZoneProcess(joystick.getRawAxis(1));
        final double joystickZ = 1 * Tools.deadZoneProcess(joystick.getRawAxis(2));
        SmartDashboard.putNumber("Axis1", joystickX);
        SmartDashboard.putNumber("Axis2", joystickY);
        SmartDashboard.putNumber("Axis3", joystickZ);
        SmartDashboard.putNumber("Axis4", joystick.getRawAxis(3));

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

        if (joystick.getRawButton(3)) {
            // 手首の位置をリセット
            State.Hand.rotateState = RotateState.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            // 手首が右回転する
            State.Hand.rotateState = RotateState.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            // 手首が左回転する
            State.Hand.rotateState = RotateState.s_leftRotateHand;
        } else if (joystick.getRawButton(4)) {
            // 手首が180°回転する
            State.Hand.rotateState = RotateState.s_moveHandToSpecifiedAngle;
        }
        if (joystick.getRawButtonPressed(4)) {
            // 手首が180°回転する
            State.Hand.targetAngle = State.Hand.actualHandAngle + 180;
            State.Hand.isResetHandPID = true;
        }

        SmartDashboard.putBoolean("test", joystick.getRawButton(5));

        if (getSeveralRawButtonPressed(new int[]{2, 6, 7, 8, 9, 10, 11, 12}) || getSeveralRawButtonReleased(new int[]{2, 6, 7, 8, 9, 10, 11, 12})) {
            State.Arm.resetPidController = true;
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth;
        }

        if (getSeveralRawButton(new int[]{7, 8, 9, 10, 11, 12})) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
        }

        boolean enableLimelight = true && driveController.getPOV() == 0;
        if (joystick.getRawAxis(3) < -0.8) {
            // 各アームの角度をコントローラーで変える -> もっとも直感的
            State.Arm.state = State.Arm.States.s_moveArmMotor;
            State.Arm.rootSpeed = joystickX;
            State.Arm.jointSpeed = joystickY;
        } else if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            if (State.Arm.actualHeight < -10) {
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
            } else {
                State.Arm.targetHeight = Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
                State.Arm.targetDepth = enableLimelight ? State.limelightToBackGoal - Const.Calculation.Limelight.LimelightToArm : State.Arm.TargetDepth.TopCorn;
            }
        } else if (joystick.getRawButton(9)) {
            // 真ん中のコーンのゴールまでアームを伸ばす
            if (State.Arm.actualHeight < -10) {
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
            } else {
                State.Arm.targetHeight = Const.Calculation.Limelight.MiddleGoalHeight - Const.Arm.RootHeightFromGr;
                State.Arm.targetDepth = enableLimelight ? State.limelightToFrontGoal - Const.Calculation.Limelight.LimelightToArm : State.Arm.TargetDepth.MiddleCorn;
            }
        } else if (joystick.getRawButton(11)) {
            // 前のコーンのゴールまでアームを伸ばす
                State.Arm.targetHeight = Const.Calculation.Limelight.BottomGoalHeight - Const.Arm.RootHeightFromGr;
                State.Arm.targetDepth = State.Arm.TargetDepth.BottomCorn;
        } else if (joystick.getRawButton(8)) {
            // 奥のキューブのゴールまでアームを伸ばす
            if (State.Arm.actualHeight < -10) {
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
            } else {
                State.Arm.targetHeight = Const.Calculation.Camera.TopGoalHeight - Const.Arm.RootHeightFromGr;
                State.Arm.targetDepth = State.Arm.TargetDepth.TopCube;
            }
        } else if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            if(State.Arm.actualHeight < -10) {
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
            } else {
                State.Arm.targetHeight = Const.Calculation.Camera.MiddleGoalHeight - Const.Arm.RootHeightFromGr;
                State.Arm.targetDepth = State.Arm.TargetDepth.MiddleCube;
            }
        } else if (joystick.getRawButton(12)) {
            // 前のキューブのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Camera.BottomGoalHeight - Const.Arm.RootHeightFromGr;
            State.Arm.targetDepth = State.Arm.TargetDepth.BottomCube;
        } else if (driveController.getPOV() == 90) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = -7;
            State.Arm.targetDepth = State.Arm.TargetDepth.SubStation;
        } else {
            if (joystick.getPOV() == 0) {
                adjustArmPosition(0, Const.Arm.TargetModifyRatio);
            } else if (joystick.getPOV() == 180) {
                adjustArmPosition(0, -Const.Arm.TargetModifyRatio);
            } else if (joystick.getPOV() == 90) {
                adjustArmPosition(Const.Arm.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 270) {
                adjustArmPosition(-Const.Arm.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 45) {
                adjustArmPosition(Const.Arm.TargetModifyRatio, Const.Arm.TargetModifyRatio);
            } else if (joystick.getPOV() == 135) {
                adjustArmPosition(Const.Arm.TargetModifyRatio, -Const.Arm.TargetModifyRatio);
            } else if (joystick.getPOV() == 225) {
                adjustArmPosition(-Const.Arm.TargetModifyRatio, -Const.Arm.TargetModifyRatio);
            } else if (joystick.getPOV() == 315) {
                adjustArmPosition(-Const.Arm.TargetModifyRatio, Const.Arm.TargetModifyRatio);
            }
        }

        if (joystick.getRawAxis(3) < -0.8) {
            State.Arm.state = State.Arm.States.s_moveArmMotor;
            State.Arm.rootSpeed = joystickX;
            State.Arm.jointSpeed = joystickY;
            // 奥のコーン
        } else if (joystick.getRawButton(7)) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    if (State.Arm.targetHeight < 50) {
                        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                        State.Arm.targetHeight = Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
                        State.Arm.targetDepth = enableLimelight ? State.limelightToBackGoal - Const.Calculation.Limelight.LimelightToArm : State.Arm.TargetDepth.TopCorn;
                        State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    }
                    break;
            }
        } else if (joystick.getRawButton(9)) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    if (State.Arm.targetHeight < 50) {
                        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                        State.Arm.targetHeight = Const.Calculation.Limelight.MiddleGoalHeight - Const.Arm.RootHeightFromGr;
                        State.Arm.targetDepth = enableLimelight ? State.limelightToFrontGoal - Const.Calculation.Limelight.LimelightToArm : State.Arm.TargetDepth.MiddleCorn;
                        State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    }
                    break;
            }

        } else if (joystick.getRawButton(8)) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    if (State.Arm.targetHeight < 50) {
                        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                        State.Arm.targetHeight = Const.Calculation.Camera.TopGoalHeight - Const.Arm.RootHeightFromGr;
                        State.Arm.targetDepth = State.Arm.TargetDepth.TopCube;
                        State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    }
                    break;
            }
        } else if (joystick.getRawButton(10)) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = DriveMode.GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    if (State.Arm.targetHeight < 50) {
                        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                        State.Arm.targetHeight = Const.Calculation.Camera.MiddleGoalHeight - Const.Arm.RootHeightFromGr;
                        State.Arm.targetDepth = State.Arm.TargetDepth.MiddleCube;
                        State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    }
                    break;
            }
        }

        if (joystick.getRawButton(2)) {
            // すべてBasicPositionに戻る
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.Hand.rotateState = RotateState.s_turnHandBack;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
        }

        if (driveController.getBButton()) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_limelightTracking;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = Tools.calculateAngles(State.Arm.targetDepth, State.Arm.targetHeight);
        Double target = targetAngles.get("RootAngle");
        if (target != null) {
            State.Arm.targetRootAngle = target;
        } else {
            State.Arm.targetRootAngle = State.Arm.actualRootAngle;
        }
        target = targetAngles.get("JointAngle");
        if (target != null) {
            State.Arm.targetJointAngle = target;
        } else {
            State.Arm.targetJointAngle = State.Arm.actualJointAngle;
        }

    }

    /**
     * @param Height : ターゲットのX座標[cm]
     * @param Depth  : ターゲットのZ座標[cm]
     *               この関数に座標の値域を記述する
     * @return 入力の座標が正しいか[boolean]
     */
    private static boolean isNewTargetPositionInLimit(double Height, double Depth) {
        double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

        boolean isInOuterBorder = length < Const.Arm.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > Const.Arm.TargetPositionInnerLimit;
        boolean isInDepthLimit = Depth > -23;

        SmartDashboard.putBoolean("InLimit", isInOuterBorder && isOutInnerBorder && isInDepthLimit);

        // TODO XButtonでコントロールする時のターゲット座標の制限を考える
        return isInOuterBorder && isOutInnerBorder && isInDepthLimit;
    }

    /**
     * 複数のボタンのうちどれかが押されているか判定する
     * 複数のボタンのgetButtonをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonでtrueかどうか
     */
    private boolean getSeveralRawButton(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButton(buttonIdx);
        }
        return flag;
    }

    /**
     * 複数のボタンのうちどれかが押されたか判定する
     * 複数のボタンのgetButtonPressedをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonPressedでtrueかどうか
     */
    private boolean getSeveralRawButtonPressed(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonPressed(buttonIdx);
        }
        return flag;
    }

    /**
     * 複数のボタンのうちどれかが離されたか判定する
     * 複数のボタンのgetButtonReleasedをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonReleasedでtrueかどうか
     */
    private boolean getSeveralRawButtonReleased(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonReleased(buttonIdx);
        }
        return flag;
    }

    static void adjustArmPosition(double diffH, double diffD) {
        State.Arm.state = State.Arm.States.s_adjustArmPosition;
        if (isNewTargetPositionInLimit(State.Arm.targetHeight + diffH, State.Arm.targetDepth + diffD)) {
            State.Arm.targetHeight += diffH;
            State.Arm.targetDepth += diffD;
        } else {
            State.Arm.targetHeight = State.Arm.actualHeight;
            State.Arm.targetDepth = State.Arm.actualDepth;
        }
    }

    enum GrabGamePiecePhase {
        //basicPositionに移動する
        Phase1,
        //ハンドを開ける, アームを下げる
        Phase2,
        //ハンドを閉める
        Phase3,
        //アームを上げる
        Phase4,
        Phase5,
        Phase6,
        Phase7,
        Phase8,
    }

}