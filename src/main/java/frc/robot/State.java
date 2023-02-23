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
    public static IntakeState intakeState;
    public static HandState handState;
    public static ArmState armState;

    public static double armTargetHeight;
    public static double armTargetDepth;

    public static double armActualRootAngle, armActualJointAngle;

    public static double armTargetRootAngle, armTargetJointAngle;

    public static double armActualHeight;
    public static double armActualDepth;

    public static double leftY;
    public static double rightX;
    public static double armUnderMotorFeedforward;
    public static double armTopMotorFeedforward;
    public static boolean isArmAtTarget;
    public static boolean resetArmPidController;
    public static boolean resetArmEncoder;

    public static double limelightTargetHeight, limelightTargetDepth;

    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        handState = HandState.s_releaseHand;

        //init armMode value
        armTargetHeight = 0.0;
        armTargetDepth = 0.0;

        armActualHeight = 0.0;
        armActualDepth = 0.0;

        armTargetRootAngle = 0.0;
        armTargetJointAngle = 0.0;

        armActualRootAngle = 0.0;
        armActualJointAngle = 0.0;

        leftY = 0.0;
        rightX = 0.0;

        limelightTargetHeight = 10.0;
        limelightTargetDepth = 80.0;

        armUnderMotorFeedforward = 0.0;
        armTopMotorFeedforward = 0.0;

        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = IntakeState.s_stopConveyor;
        armState = ArmState.s_fixArmPosition;
        isArmAtTarget = false;
        resetArmPidController = false;
        resetArmEncoder = false;
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
