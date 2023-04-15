package frc.robot.phase;

import frc.robot.consts.CameraConst;
import frc.robot.consts.LimelightConst;
import frc.robot.states.*;
import frc.robot.consts.ArmConst;
import frc.robot.mode.ArmMode;
import frc.robot.subClass.Util;

public class Autonomous {
    /**
     * コーンをゴールに入れる<br>
     * バックする
     */
    private static PhaseTransition phaseTransitionA;
    /**
     * キューブをゴールに入れる<br>
     * バックする
     */
    private static PhaseTransition phaseTransitionB;
    /**
     * なし
     */
    private static PhaseTransition phaseTransitionC;

    /**
     * アームをtarget[Height/Depth]に動かす
     * s_moveArmToSpecifiedPosition
     * isAtTarget()で終了
     *
     * @param targetHeight ターゲットの高さ[cm]
     * @param targetDepth  ターゲットの奥行き[cm]
     * @param phaseName    出力されるフェーズの名前
     */
    private static PhaseTransition.Phase moveArmTo(double targetHeight, double targetDepth, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = targetHeight;
                    ArmState.targetDepth = targetDepth;
                },
                (double time) -> {
                    return ArmState.isAtTarget();
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * リレーポイントまで動かす
     * （一度でも閾を超えたら終了）
     *
     * @param relayHeight リレーポイントの高さ[cm]
     * @param relayDepth  リレーポイントの奥行き[cm]
     * @param phaseName   出力されるフェーズの名前
     */
    private static PhaseTransition.Phase relayArmTo(double relayHeight, double relayDepth, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = relayHeight;
                    ArmState.targetDepth = relayDepth;
                },
                (double time) -> {
                    return Util.Calculate.isOverRelayToGoal(ArmState.actualHeight, ArmState.actualDepth);
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * adjustArmPositionで位置の調整を行う
     * 一定時間[sec]アームをある向きに動かす
     *
     * @param diffH     高さ方向の時間[20ms]あたりの変化量[cm]
     * @param diffD     奥行き方向の時間[20ms]あたりの変化量[cm]
     * @param waiter    動かす時間（実行時間）[sec]
     * @param phaseName 出力されるフェーズの名前
     */
    private static PhaseTransition.Phase adjustArmTo(double diffH, double diffD, double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    ArmState.armState = ArmState.ArmStates.s_adjustArmPosition;
                    ArmMode.adjustArmPosition(diffH, diffD);
                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );

    }

    /**
     * ハンドを一定時間[sec]開く！！
     *
     * @param waiter    ハンドを開く時間（実行時間）[sec]
     * @param phaseName 出力されるフェーズの名前
     */
    public static PhaseTransition.Phase releaseHand(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * 一定距離走行する[cm]
     *
     * @param targetMeter 走行する距離[cm]
     * @param phaseName   出力されるフェーズの名前
     */
    public static PhaseTransition.Phase pidDriveTo(double targetMeter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    DriveState.driveState = DriveState.DriveStates.s_pidDrive;
                    DriveState.targetMeter = targetMeter;
                },
                (double time) -> {
                    return time > targetMeter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * 一定のスピードで一定時間[sec]走行する
     *
     * @param waiter    走行する時間（実行時間）[sec]
     * @param xSpeed    前進スピード
     * @param zRotation 回転スピード
     * @param phaseName 出力されるフェーズの名前
     */
    private static PhaseTransition.Phase midDriveTo(double waiter, double xSpeed, double zRotation, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    DriveState.driveState = DriveState.DriveStates.s_midDrive;
                    DriveState.xSpeed = xSpeed;
                    DriveState.zRotation = zRotation;
                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * outTakeを一定時間[sec]行う
     *
     * @param waiter    outTakeする時間（実行時間）[sec]
     * @param phaseName 出力されるフェーズの名前
     */
    private static PhaseTransition.Phase outTake(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
                },
                (double time) -> {
                    return time > waiter;
                },
                phaseName
        );
    }

    /**
     * BasicPositionにする
     * アームの位置、アームを真ん中に、ハンドを初期位置に
     *
     * @param waiter    戻るのを待つ時間（実行時間）[sec]
     * @param phaseName 出力されるフェーズの名前
     */
    private static PhaseTransition.Phase basicPosition(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    Util.Calculate.setInitWithRelay();
                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    /**
     * 初期化（PIDやEncoder、Sensor等ののカウントをリセット！！！）
     *
     * @param waiter    待機時間（実行時間）[sec]
     * @param phaseName 出力されるフェーズの名前
     */
    private static PhaseTransition.Phase Init(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {

                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                phaseName
        );
    }

    public static void autonomousInit() {
        phaseTransitionA = new PhaseTransition();
        phaseTransitionB = new PhaseTransition();
        phaseTransitionC = new PhaseTransition();
        PhaseTransition.Phase.PhaseInit();

        phaseTransitionA.registerPhase(
                // コーン！！
                // 初期化中につき待機！
                Init(1, "Initializing And Waiting..."),
                // アームをBasicPositionに
                basicPosition(5, "Reset To BasicPosition"),
                // アームをMiddleのコーンのゴールまで伸ばす
                relayArmTo(ArmConst.RelayPointToGoalHeight, ArmConst.RelayPointToGoalDepth, "Move Arm To RelayPoint"),
                moveArmTo(LimelightConst.MiddleGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.MiddleCone,
                        "Move Arm To MiddleConeGoal"),
                // ハンドを開いて、コーンを置く
                releaseHand(3, "Release Hand"),
                // バックする
                pidDriveTo(-2, "Drive Back"),
                // アームをBasicPositionに
                basicPosition(5, "Reset To BasicPosition")


                // basicArmTo(ArmConst.InitialHeight, ArmConst.InitialDepth, "move arm to basic position"),
                // adjustArmTo(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio, 2, "move to target"),
                // adjustArmTo(0, ArmConst.TargetModifyRatio, 3, "move foward")
                // relayArmTo(GrabGamePiecePhaseConst.armRelayPointHeight, GrabGamePiecePhaseConst.armRelayPointDepth, "move arm to relay point"),
                // moveArmTo( CameraConst.GoalHeight - ArmConst.RootHeightFromGr, State.armToTag, "move arm to cube goal"),
                // releaseHand(2, "release cone")
                // drive(-1, 2, "move to target")
                // driveTo(-3, "move to target")

        );

        phaseTransitionB.registerPhase(
                // キューブ！！
                // 初期化中につき待機！
                Init(1, "Initializing And Waiting..."),
                // アームをBasicPositionに
                basicPosition(5, "Reset To BasicPosition"),
                // アームをTopのキューブのゴールまで伸ばす
                relayArmTo(ArmConst.RelayPointToGoalHeight, ArmConst.RelayPointToGoalDepth, "Move Arm To RelayPoint"),
                moveArmTo(CameraConst.TopGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.TopCube,
                        "Move Arm To TopCubeGoal"),
                // ハンドを開いて、キューブを置く
                releaseHand(3, "Release Hand"),
                // バックする
                pidDriveTo(-2,  "Drive Back"),
                // アームをBasicPositionに
                basicPosition(5, "Reset To BasicPosition")

//            moveArmTo(ArmConst.InitialHeight, ArmConst.InitialDepth, "move arm to basic position")
//            relayArmTo(ArmConst.RelayPointToGoalHeight, ArmConst.RelayPointToGoalDepth, "move arm to relay point"),
//            moveArmTo(  CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.MiddleCube, "move arm to cube goal"),
//            releaseHand(2, "release cube")
                // drive(-1, 2, "move to target")
                // driveTo(-3, "move to target")
        );

        phaseTransitionC.registerPhase(
                outTake(5, "GamePiece Outtake"),
                pidDriveTo(-2, "Drive Back")
        );
    }

    public static void run() {
        switch (State.autonomousPhaseTransType) {
            case "A":
                phaseTransitionA.run();
                break;
            case "B":
                phaseTransitionB.run();
                break;
            case "C":
                phaseTransitionC.run();
                break;
            default:
                phaseTransitionC.run();
                System.out.println("Now Running TransitionC!\nThere's something wrong\nPlease Enter Autonomous Phase Transition Type!!!");
                break;
        }
    }
}
