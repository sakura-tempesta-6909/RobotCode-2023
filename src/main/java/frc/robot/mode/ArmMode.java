package frc.robot.mode;

import frc.robot.component.Arm;
import frc.robot.component.Hand;
import frc.robot.states.*;
import frc.robot.states.LimelightState.States;
import frc.robot.consts.ArmConst;
import frc.robot.consts.CameraConst;
import frc.robot.consts.GrabGamePiecePhaseConst;
import frc.robot.consts.LimelightConst;
import frc.robot.subClass.Tools;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subClass.Util;

public class ArmMode extends Mode {
    /**
     * StartButton - DriveModeに
     */
    @Override
    public void changeMode() {
        if (driveController.getStartButton()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getPOV() == 0) {
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

        DriveState.xSpeed = -driveController.getLeftY();
        DriveState.zRotation = -driveController.getRightX();
        DriveState.driveState = DriveState.DriveStates.s_midDrive;

        LimelightState.isLimelightOn = true;
        LimelightState.limelightState = States.s_tapeDetection;

        if (driveController.getAButtonPressed()) {
            if (IntakeState.intakeExtensionState == IntakeState.IntakeExtensionStates.s_closeIntake){
                IntakeState.intakeExtensionState = IntakeState.IntakeExtensionStates.s_openIntake;
            } else{
                IntakeState.intakeExtensionState = IntakeState.IntakeExtensionStates.s_closeIntake;
            }
            
        }

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
        } else {
            IntakeState.intakeState = IntakeState.RollerStates.s_stopRoller;
        }

        final double joystickX = -1 * Tools.deadZoneProcess(joystick.getRawAxis(0));
        final double joystickY = 1 * Tools.deadZoneProcess(joystick.getRawAxis(1));
        final double joystickZ = 1 * Tools.deadZoneProcess(joystick.getRawAxis(2));

        if (driveController.getRightBumper() && driveController.getLeftBumper()) {
            // アームの位置をリセット
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
        } else if (joystickZ > 0.5) {
            // アームを右に動かす
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_moveRightMotor;
        } else if (joystickZ < -0.5) {
            // アームを左に動かす
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_moveLeftMotor;
        }

        if (joystick.getRawButton(1)) {
            // ハンドを開く
            HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
        }

        if (joystick.getRawButton(3)) {
            // 手首の位置をリセット
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            // 手首が右回転する
            HandState.rotateState = HandState.RotateStates.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            // 手首が左回転する
            HandState.rotateState = HandState.RotateStates.s_leftRotateHand;
        } else if (joystick.getRawButton(4)) {
            // 手首が180°回転する
            HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;
        }
        if (joystick.getRawButtonPressed(4)) {
            // 手首が180°回転する
            HandState.targetAngle = HandState.actualHandAngle + 180;
            HandState.isResetHandPID = true;
        }

        if (getSeveralRawButtonPressed(new int[]{2, 6, 7, 8, 9, 10, 11, 12}) || getSeveralRawButtonReleased(new int[]{2, 6, 7, 8, 9, 10, 11, 12})) {
            ArmState.resetPidController = true;
            ArmState.targetHeight = ArmState.actualHeight;
            ArmState.targetDepth = ArmState.actualDepth;
            ArmState.relayToGoalOver = false;
            ArmState.relayToInitOver = false;
            ArmState.targetToGoalOver = false;
        }

        if (getSeveralRawButton(new int[]{7, 8, 9, 10, 11, 12})) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
        }

        if (joystick.getRawAxis(3) < -0.8) {
            // 各アームの角度をコントローラーで変える -> もっとも直感的
            ArmState.armState = ArmState.ArmStates.s_moveArmMotor;
            ArmState.rootSpeed = joystickX;
            ArmState.jointSpeed = joystickY;
        } else if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            Util.Calculate.setGoalWithRelay(
                    LimelightConst.TopGoalHeight - ArmConst.RootHeightFromGr,
                    ArmState.TargetDepth.TopCorn
            );
        } else if (joystick.getRawButton(9)) {
            // 真ん中のコーンのゴールまでアームを伸ばす
            Util.Calculate.setGoalWithRelay(
                    LimelightConst.MiddleGoalHeight - ArmConst.RootHeightFromGr,
                    ArmState.TargetDepth.MiddleCorn
            );
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = LimelightConst.MiddleGoalHeight - ArmConst.RootHeightFromGr;
                if(LimelightState.tv) {
                    if (50 < LimelightState.armToGoal && LimelightState.armToGoal < 120) {
                        ArmState.targetDepth = LimelightState.armToGoal;
                        if (ArmState.isAtTarget() && ArmState.isAtLeftAndRightTarget()) {
                            HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                        }
                    }

                } else {
                    ArmState.targetDepth = ArmState.TargetDepth.MiddleCorn;
                }
            }
        } else if (joystick.getRawButton(11)) {
            // 前のコーンのゴールまでアームを伸ばす
            ArmState.targetHeight = LimelightConst.BottomGoalHeight - ArmConst.RootHeightFromGr;
            ArmState.targetDepth = ArmState.TargetDepth.BottomCorn;
        } else if (joystick.getRawButton(8)) {
            // 奥のキューブのゴールまでアームを伸ばす
            Util.Calculate.setGoalWithRelay(
                    CameraConst.TopGoalHeight - ArmConst.RootHeightFromGr,
                    ArmState.TargetDepth.TopCube
            );
        } else if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            Util.Calculate.setGoalWithRelay(
                    CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr,
                    ArmState.TargetDepth.MiddleCube
            );
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = CameraConst.TopGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.TopCube;
            }
        } else if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            if(!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.MiddleCube;
            }
        } else if (joystick.getRawButton(12)) {
            // 前のキューブのゴールまでアームを伸ばす
            ArmState.targetHeight = CameraConst.BottomGoalHeight - ArmConst.RootHeightFromGr;
            ArmState.targetDepth = ArmState.TargetDepth.BottomCube;
            if (ArmState.targetToGoalOver) {
                HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
            }
        } else if (driveController.getPOV() == 90) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            ArmState.targetHeight = ArmConst.SubStationHeight;
            ArmState.targetDepth = ArmState.TargetDepth.SubStation;
        } else {
            if (joystick.getPOV() == 0) {
                adjustArmPosition(0, ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 180) {
                adjustArmPosition(0, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 90) {
                adjustArmPosition(ArmConst.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 270) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 45) {
                adjustArmPosition(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 135) {
                adjustArmPosition(ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 225) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 315) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            }
        }

        if (joystick.getRawAxis(3) < -0.8) {
            ArmState.armState = ArmState.ArmStates.s_moveArmMotor;
            ArmState.rootSpeed = joystickX;
            ArmState.jointSpeed = joystickY;
        }

        if (joystick.getRawButton(2)) {
            // すべてBasicPositionに戻る
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
            Util.Calculate.setInitWithRelay();
        }

        if (driveController.getBButton()) {
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
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

        boolean isInOuterBorder = length < ArmConst.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > ArmConst.TargetPositionInnerLimit;

        SmartDashboard.putBoolean("InLimit", isInOuterBorder && isOutInnerBorder);

        return isInOuterBorder && isOutInnerBorder;
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

    /**
     * アームの微調整を行う
     * ちょっとずつターゲットを変える
     *
     * @param diffH 高さ方向の時間[20ms]あたりの変化量[cm]
     * @param diffD 奥行き方向の時間[20ms]あたりの変化量[cm]
     */
    public static void adjustArmPosition(double diffH, double diffD) {
        ArmState.armState = ArmState.ArmStates.s_adjustArmPosition;
        if (isNewTargetPositionInLimit(ArmState.targetHeight + diffH, ArmState.targetDepth + diffD)) {
            ArmState.targetHeight += diffH;
            ArmState.targetDepth += diffD;
        } else {
            ArmState.targetHeight = ArmState.actualHeight;
            ArmState.targetDepth = ArmState.actualDepth;
        }
    }
}