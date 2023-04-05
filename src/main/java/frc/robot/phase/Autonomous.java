package frc.robot.phase;

import frc.robot.consts.LimelightConst;
import frc.robot.states.*;
import frc.robot.consts.ArmConst;
import frc.robot.mode.ArmMode;
import frc.robot.subClass.Util;

public class Autonomous {
    private static PhaseTransition phaseTransitionA;
    private static PhaseTransition phaseTransitionB;
    private static PhaseTransition phaseTransitionC;

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
    public static PhaseTransition.Phase driveWithPosition(double targetMeter, String phaseName) {
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
    private static PhaseTransition.Phase driveWithTime(double xSpeed, double zRotation, double waiter, String phaseName) {
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

    public static void autonomousInit() {
        phaseTransitionA = new PhaseTransition();
        phaseTransitionB = new PhaseTransition();
        phaseTransitionC = new PhaseTransition();
        PhaseTransition.Phase.PhaseInit();

        phaseTransitionA.registerPhase(
                new PhaseTransition.Phase(
                        () -> {

                        },
                        (double time) -> {
                            return time > 1;
                        },
                        () -> {
                            DriveState.resetPIDController = true;
                            DriveState.resetPosition = true;
                            ArmState.resetPidController = true;
                        },
                        "Waiting..."
                ),
                // アームをBasicPositionに
                basicPosition(5, "Reset To BasicPosition"),
                // アームをコーンのゴールまで伸ばす
                relayArmTo(ArmConst.RelayPointToGoalHeight, ArmConst.RelayPointToGoalDepth, "Move Arm To RelayPoint"),
                moveArmTo(LimelightConst.MiddleGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.MiddleCorn,
                        "Move Arm To MiddleCornGoal"),
                // ハンドを開いて、キューブを置く
                releaseHand(3, "Release Hand"),
                // バックする
                driveWithTime(-1, 0, 2, "Drive Back"),
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
                new PhaseTransition.Phase(
                        () -> {

                        },
                        (double time) -> {
                            return time > 10;
                        },
                        () -> {
                            DriveState.resetPIDController = true;
                            DriveState.resetPosition = true;
                            ArmState.resetPidController = true;
                        },
                        "wait"
                )
//            moveArmTo(ArmConst.InitialHeight, ArmConst.InitialDepth, "move arm to basic position")

//            relayArmTo(ArmConst.RelayPointToGoalHeight, ArmConst.RelayPointToGoalDepth, "move arm to relay point"),
//            moveArmTo(  CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.MiddleCube, "move arm to cube goal"),
//            releaseHand(2, "release cube")
                // drive(-1, 2, "move to target")
                // driveTo(-3, "move to target")
        );

        phaseTransitionC.registerPhase(
                // drive(1, 0.5, "drive to target"),
                //     new PhaseTransition.Phase(
                //             () -> {
                //                 DriveState.resetPIDController = true;
                //                 DriveState.resetPosition = true;
                //                 DriveState.state = DriveState.States.s_midDrive;
                //                 DriveState.xSpeed = -1;
                //             },
                //             (double time) -> {
                //                 return time > 2;
                //             }
                //     )
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
                System.out.println("Please Enter Autonomous Phase Transition Type");
                break;
        }
    }
}
