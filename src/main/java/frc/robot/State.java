package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.mode.DriveMode;
import frc.robot.mode.Mode;
import frc.robot.mode.TestMode;
import frc.robot.subClass.Const;

public class State {
    public static Modes mode;
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;
    public static RollerState intakeState;
    public static IntakeExtensionState intakeExtensionState;
    public static GrabHandState grabHandState;
    public static RotateHandState rotateHandState;
    public static MoveArmState moveArmState;
    public static AdjustArmState adjustArmState;
    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        grabHandState = GrabHandState.s_releaseHand;
        intakeExtensionState = IntakeExtensionState.s_openIntake;
        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = RollerState.s_stopRoller;
        moveArmState = MoveArmState.s_fixArmPosition;
        rotateHandState = RotateHandState.s_stopHand;
        adjustArmState = AdjustArmState.s_stopArmSideMovement;
    }

    public enum DriveState {
        /** ロボットの速度を速くする */
        s_fastDrive,
        /** ロボットの速度を中くらいにする */
        s_midDrive,
        /** ロボットの速度を遅くする */
        s_slowDrive,
        /** ロボットの速度を0にする */
        s_stopDrive,

    }

    public enum RollerState {
        /** Rollerを外向きに動かし、ゲームピースを出す */
        s_outtakeGamePiece,
        /** Rollerを内向きに動かし、ゲームピースを取り込む */
        s_intakeGamePiece,
        /** Rollerの動きを止める */
        s_stopRoller,

    }

    public enum IntakeExtensionState {
        /** Intakeを出す */
        s_openIntake,
        /** Intakeをしまう */
        s_closeIntake,
    }

    public enum GrabHandState {
        /** 物体をつかむ */
        s_grabHand,
        /** 物体を離す */
        s_releaseHand,
    }

    public enum RotateHandState {
        /** 手首を回転させる */
        s_rotateHand,
        /** 手首の回転を止める */
        s_stopHand,
    }

    public enum MoveArmState {
        /** アームを指定した場所に移動させる */
        s_moveArmToSpecifiedPosition,
        /** アームの支点を動かす */
        s_moveArmMotor,
        /** アームをその場で固定する */
        s_fixArmPosition,
    }

    public enum AdjustArmState {
        /** アームを左右に動かす */
        s_moveArmLeftOrRight,
        /** アームの左右の動きを止める */
        s_stopArmSideMovement,
    }

    public enum Modes {
        k_drive(new DriveMode()),
        k_test(new TestMode());

        private final Mode mode;
        Modes(Mode mode) {
            this.mode = mode;
        }

        public void changeMode() {
            this.mode.changeMode();
        }

        public void changeState() {
            this.mode.changeState();
        }
    }
}
