package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mode.Mode;
import frc.robot.SubClass.Const;

public class State {
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;
    public static IntakeState intakeState;
    public static HandState handState;
    public static ArmState armState;
    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        handState = HandState.s_releaseHand;
        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = IntakeState.s_stopConveyor;
        armState = ArmState.s_fixArmPosition;
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

    public enum IntakeState {
        /** インテイクを外向きに動かす */
        s_outtakeConveyor,
        /** インテイクを内向きに動かす */
        s_intakeConveyor,
        /** インテイクの動きを止める */
        s_stopConveyor,

    }

    public enum HandState {
        /** 物体をつかむ */
        s_grabHand,
        /** 物体を離す */
        s_releaseHand,
    }

    public enum ArmState {
        /** アームを指定した場所に移動させる */
        s_moveArmToSpecifiedPosition,
        /** アームの支点を動かす */
        s_moveArmMotor,
        /** アームをその場で固定する */
        s_fixArmPosition,
    }
}
