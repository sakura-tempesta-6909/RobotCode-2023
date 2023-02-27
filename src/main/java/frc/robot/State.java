package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.mode.DriveMode;
import frc.robot.mode.Mode;
import frc.robot.mode.TestMode;
import frc.robot.subClass.Const;

import java.util.HashMap;
import java.util.Map;

public class State {
    public static Modes mode;
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;

    public static double limelightTrackingZRotation;
    public static double distanceFromLimelightToGoal; // [cm]


    public static IntakeState intakeState;
    public static HandState handState;

    public static MoveLeftAndRightArmState moveLeftAndRightArmState;

    public static class Arm {
        public static States state;
        public static double targetHeight;
        public static double targetDepth;

        public static double actualRootAngle, actualJointAngle;

        public static double targetRootAngle, targetJointAngle;

        public static double actualHeight;
        public static double actualDepth;

        public static double leftY;
        public static double rightX;

        public static double moveLeftAndRightMotor;

        public static double underMotorFeedforward;
        public static double topMotorFeedforward;
        public static boolean isArmAtTarget;
        public static boolean resetArmPidController;
        public static boolean resetArmEncoder;

        public static double limelightTargetHeight, limelightTargetDepth;

        public enum States {
            /**
             * アームを指定した場所に移動させる
             */
            s_moveArmToSpecifiedPosition,
            /**
             * アームの支点を動かす
             */
            s_moveArmMotor,
            /**
             * アームをその場で固定する
             */
            s_fixArmPosition,
        }
    }

    public static Map<String, Double> voltage = new HashMap<>();
    public static double distanceFromCameraToTag; // [cm]
    public static double aprilTagAngleHeight;
    public static double aprilTagAngleWidth;
    public static double distanceFromArmToTag; // [cm]
    public static double distanceFromArmToGoal; // [cm]

    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        handState = HandState.s_releaseHand;

        //init armMode value
        Arm.targetHeight = 0.0;
        Arm.targetDepth = 0.0;

        Arm.actualHeight = 0.0;
        Arm.actualDepth = 0.0;

        Arm.targetRootAngle = 0.0;
        Arm.targetJointAngle = 0.0;

        Arm.actualRootAngle = 0.0;
        Arm.actualJointAngle = 0.0;

        Arm.leftY = 0.0;
        Arm.rightX = 0.0;

        Arm.limelightTargetHeight = 10.0;
        Arm.limelightTargetDepth = 80.0;

        Arm.underMotorFeedforward = 0.0;
        Arm.topMotorFeedforward = 0.0;

        Arm.moveLeftAndRightMotor = 0.0;

        voltage = new HashMap<>();
        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = IntakeState.s_stopConveyor;
        Arm.state = Arm.States.s_fixArmPosition;
        Arm.isArmAtTarget = false;
        Arm.resetArmPidController = false;
        Arm.resetArmEncoder = false;
    }

    public enum DriveState {
        /**
         * ロボットの速度を速くする
         */
        s_fastDrive,
        /**
         * ロボットの速度を中くらいにする
         */
        s_midDrive,
        /**
         * ロボットの速度を遅くする
         */
        s_slowDrive,
        /**
         * ロボットの速度を0にする
         */
        s_stopDrive,
        // targetに照準を合わせる
        s_targetTracking,


    }

    public enum IntakeState {
        /**
         * インテイクを外向きに動かす
         */
        s_outtakeConveyor,
        /**
         * インテイクを内向きに動かす
         */
        s_intakeConveyor,
        /**
         * インテイクの動きを止める
         */
        s_stopConveyor,

    }

    public enum HandState {
        /**
         * 物体をつかむ
         */
        s_grabHand,
        /**
         * 物体を離す
         */
        s_releaseHand,
    }

    public enum MoveLeftAndRightArmState{
        /**
         * アームを左右に動かす
         */
        s_moveLeftAndRightMotor,
        /**
         * アームを固定する
         */
        s_fixLeftAndRightMotor,
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
