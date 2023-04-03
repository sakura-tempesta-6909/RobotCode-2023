package frc.robot.phase;

import frc.robot.component.Intake;
import frc.robot.states.*;
import frc.robot.consts.ArmConst;
import frc.robot.consts.CameraConst;
import frc.robot.consts.GrabGamePiecePhaseConst;
import frc.robot.mode.ArmMode;
import frc.robot.subClass.Util;

public class Autonomous {
    private static PhaseTransition phaseTransitionA;
    private static PhaseTransition phaseTransitionB;
    private static PhaseTransition phaseTransitionC;

    private static PhaseTransition.Phase basicArmTo(double targetHeight, double targetDepth, String phaseName) {
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
                ArmState.resetPidController  = true;
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
                return Util.Calculate.relayReach(ArmState.actualHeight, ArmState.actualDepth);
            },
            () -> {
                DriveState.resetPIDController = true;
                DriveState.resetPosition = true;
                ArmState.resetPidController = true;
            },
            phaseName
    );
    }

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

    public static PhaseTransition.Phase driveTo(double targetMeter, String phaseName) {
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

    private static PhaseTransition.Phase drive(double xSpeed, double waiter, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
                DriveState.driveState = DriveState.DriveStates.s_midDrive;
                DriveState.xSpeed = xSpeed;
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

    private static PhaseTransition.Phase armAdjust(double diffH, double diffD, double waiter, String phaseName) {
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

    private static PhaseTransition.Phase outtake(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
                IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
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
                    return time > 10;
                },
                () -> {
                    DriveState.resetPIDController = true;
                    DriveState.resetPosition = true;
                    ArmState.resetPidController = true;
                },
                "wait"
            ),
            // basicArmTo(ArmConst.InitialHeight, ArmConst.InitialDepth, "move arm to basic position"),
            armAdjust(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio,2,"move to target"),
            armAdjust(0, ArmConst.TargetModifyRatio, 3, "move foward")
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
            ),
            basicArmTo(ArmConst.InitialHeight, ArmConst.InitialDepth, "move arm to basic position"),
            relayArmTo(GrabGamePiecePhaseConst.armRelayPointHeight, GrabGamePiecePhaseConst.armRelayPointDepth, "move arm to relay point"),
            moveArmTo(  CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr, ArmState.TargetDepth.MiddleCube, "move arm to cube goal"),
            releaseHand(2, "release cube")
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
