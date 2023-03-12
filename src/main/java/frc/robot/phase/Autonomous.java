package frc.robot.phase;

import frc.robot.State;
import frc.robot.State.GrabHandState;
import frc.robot.subClass.Const;

public class Autonomous {
    private static PhaseTransition phaseTransitionA;
    private static PhaseTransition phaseTransitionB;
    private static PhaseTransition phaseTransitionC;

    private static PhaseTransition.Phase moveArmTo(double targetHeight, double targetDepth, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = targetHeight;
                    State.Arm.targetDepth = targetDepth;
                },
                (double time) -> {
                    return State.Arm.isAtTarget;
                },
                () -> {
                    State.Arm.resetPidController = true;
                },
                phaseName
        );
    }

    public static PhaseTransition.Phase releaseHand(double waiter, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                },
                (double time) -> {
                    return time > waiter;
                },
                phaseName
        );
    }

    public static PhaseTransition.Phase driveTo(double targetLength, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Drive.state = State.Drive.States.s_pidDrive;
                },
                (double time) -> {
                    return State.Drive.isAtTarget;
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
                moveArmTo(Const.Calculation.Camera.GoalHeight - Const.Arm.RootHeight, State.armToTag, "move arm to cube goal"),
                releaseHand(2, "release cube")
        );

        phaseTransitionB.registerPhase(
                moveArmTo(Const.Calculation.Limelight.GoalHeight - Const.Arm.RootHeight, State.armToGoal, "move arm to corn goal"),
                releaseHand(2, "release corn")
        );
    }

    public static void run() {
        switch (State.autonomousPhaseTransition) {
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
                break;
        }
    }
}
