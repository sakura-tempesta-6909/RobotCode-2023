package frc.robot.phase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.State;
import frc.robot.States.State.RollerState;
import frc.robot.States.State.Hand.RotateState;
import frc.robot.mode.ArmMode;
import frc.robot.subClass.Const;

public class Autonomous {
    private static PhaseTransition phaseTransitionA;
    private static PhaseTransition phaseTransitionB;
    private static PhaseTransition phaseTransitionC;

    private static PhaseTransition.Phase basicArmTo(double targetHeight, double targetDepth, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = targetHeight;
                State.Arm.targetDepth = targetDepth;
            },
            (double time) -> {
                return State.Arm.isAtTarget();
            },
            () -> {
                State.Drive.resetPIDController = true;
                State.Drive.resetPosition = true;
                State.Arm.resetPidController  = true;
            }, 
            phaseName
        );
    }


    private static PhaseTransition.Phase relayArmTo(double relayHeight, double relayDepth, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
        
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = relayHeight;
                    State.Arm.targetDepth = relayDepth;
                

            },
            (double time) -> {
                return State.Arm.targetHeight > Const.Arm.RelayPointHeight;
            },
            () -> {
                State.Drive.resetPIDController = true;
                State.Drive.resetPosition = true;
                State.Arm.resetPidController = true;
            },
            phaseName
    );
    }

    private static PhaseTransition.Phase moveArmTo(double targetHeight, double targetDepth, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
            
                        State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                        State.Arm.targetHeight = targetHeight;
                        State.Arm.targetDepth = targetDepth;
                    

                },
                (double time) -> {
                    return State.Arm.isAtTarget();
                },
                () -> {
                    State.Drive.resetPIDController = true;
                    State.Drive.resetPosition = true;
                    State.Arm.resetPidController = true;
                },
                phaseName
        );
    }

    public static PhaseTransition.Phase releaseHand(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                },
                (double time) -> {
                    return time > waiter;
                },
                () -> {
                    State.Drive.resetPIDController = true;
                    State.Drive.resetPosition = true;
                    State.Arm.resetPidController = true;
                },
                phaseName
        );
    }

    public static PhaseTransition.Phase driveTo(double targetMeter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Drive.state = State.Drive.States.s_pidDrive;
                    State.Drive.targetMeter = targetMeter;
                },
                (double time) -> {
                    return time > targetMeter;
                },
                () -> {
                    State.Drive.resetPIDController = true;
                    State.Drive.resetPosition = true;
                    State.Arm.resetPidController = true;
                },
                phaseName
        );
    }

    private static PhaseTransition.Phase drive(double xSpeed, double waiter, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
                State.Drive.state = State.Drive.States.s_midDrive;
                State.Drive.xSpeed = xSpeed;
            },
            (double time) -> {
                return time > waiter;
            },
            () -> {
                State.Drive.resetPIDController = true;
                State.Drive.resetPosition = true;
                State.Arm.resetPidController = true;
            },
            phaseName
        );
    }

    private static PhaseTransition.Phase armAdjust(double diffH, double diffD, double waiter, String phaseName) {
        return new PhaseTransition.Phase(
        () -> {
            State.Arm.state = State.Arm.States.s_adjustArmPosition;
            ArmMode.adjustArmPosition(diffH, diffD);
    },
    (double time) -> {
        return time > waiter;
    },
    () -> {
        State.Drive.resetPIDController = true;
        State.Drive.resetPosition = true;
        State.Arm.resetPidController = true;
    },
    phaseName
);

    }

    private static PhaseTransition.Phase outtake(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
            () -> {
                State.intakeState = RollerState.s_outtakeGamePiece;
            },
            (double time) -> {
                return time > waiter;
            },
            () -> {
                State.Drive.resetPIDController = true;
                State.Drive.resetPosition = true;
                State.Arm.resetPidController = true;
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
                    State.Drive.resetPIDController = true;
                    State.Drive.resetPosition = true;
                    State.Arm.resetPidController = true;
                },
                "wait"
            ),
            // basicArmTo(Const.Arm.InitialHeight, Const.Arm.InitialDepth, "move arm to basic position"),
            armAdjust(Const.Arm.TargetModifyRatio, Const.Arm.TargetModifyRatio,2,"move to target"),
            armAdjust(0, Const.Arm.TargetModifyRatio, 3, "move foward")
            // relayArmTo(Const.GrabGamePiecePhase.armRelayPointHeight, Const.GrabGamePiecePhase.armRelayPointDepth, "move arm to relay point"),
            // moveArmTo( Const.Calculation.Camera.GoalHeight - Const.Arm.RootHeightFromGr, State.armToTag, "move arm to cube goal"),
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
                    State.Drive.resetPIDController = true;
                    State.Drive.resetPosition = true;
                    State.Arm.resetPidController = true;
                },
                "wait"
            ),
            basicArmTo(Const.Arm.InitialHeight, Const.Arm.InitialDepth, "move arm to basic position"),
            relayArmTo(Const.GrabGamePiecePhase.armRelayPointHeight, Const.GrabGamePiecePhase.armRelayPointDepth, "move arm to relay point"),
            moveArmTo(  Const.Calculation.Camera.MiddleGoalHeight - Const.Arm.RootHeightFromGr, State.Arm.TargetDepth.MiddleCube, "move arm to cube goal"),
            releaseHand(2, "release cube")
            // drive(-1, 2, "move to target")
            // driveTo(-3, "move to target")
        );

        phaseTransitionC.registerPhase(
            // drive(1, 0.5, "drive to target"),
            //     new PhaseTransition.Phase(
            //             () -> {
            //                 State.Drive.resetPIDController = true;
            //                 State.Drive.resetPosition = true;
            //                 State.Drive.state = State.Drive.States.s_midDrive;
            //                 State.Drive.xSpeed = -1;
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
