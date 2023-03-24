package frc.robot.phase;

import java.lang.ref.PhantomReference;

import frc.robot.States.State;
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
                return State.Arm.targetHeight < Const.Arm.RelayPointHeight;
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

    public static void autonomousInit() {
        phaseTransitionA = new PhaseTransition();
        phaseTransitionB = new PhaseTransition();
        phaseTransitionC = new PhaseTransition();
        PhaseTransition.Phase.PhaseInit();

        phaseTransitionA.registerPhase(
            basicArmTo(Const.Arm.InitialHeight, Const.Arm.InitialDepth, "move arm to basic position"),
            relayArmTo(Const.GrabGamePiecePhase.armRelayPointHeight, Const.GrabGamePiecePhase.armRelayPointDepth, "move arm to relay point"),
            moveArmTo( Const.Calculation.Camera.GoalHeight - Const.Arm.RootHeightFromGr, State.armToTag, "move arm to cube goal"),
            releaseHand(2, "release cube"),
            drive(1, 4, "move to target")
            // driveTo(-3, "move to target")
                
        );

        phaseTransitionB.registerPhase(
            basicArmTo(Const.Arm.InitialHeight, Const.Arm.InitialDepth, "move arm to basic position"),
            relayArmTo(Const.GrabGamePiecePhase.armRelayPointHeight, Const.GrabGamePiecePhase.armRelayPointDepth, "move arm to relay point"),
            moveArmTo( Const.Calculation.Camera.GoalHeight - Const.Arm.RootHeightFromGr, State.armToTag, "move arm to cube goal"),
            releaseHand(2, "release cube"),
            drive(1, 4, "move to target")
            // driveTo(-3, "move to target")
        );

        phaseTransitionC.registerPhase(
                new PhaseTransition.Phase(
                        () -> {
                            State.Drive.resetPIDController = true;
                            State.Drive.resetPosition = true;
                            State.Drive.state = State.Drive.States.s_midDrive;
                            State.Drive.xSpeed = -1;
                        },
                        (double time) -> {
                            return time > 4;
                        }
                )
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
