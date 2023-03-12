package frc.robot.phase;

import frc.robot.State;
import frc.robot.State.DriveState;
import frc.robot.State.GrabHandState;
import edu.wpi.first.math.util.Units;
import frc.robot.subClass.Const;

import java.util.Objects;

public class Autonomous {
    private static PhaseTransition phaseTransitionA;
    private static PhaseTransition phaseTransitionB;
    private static PhaseTransition phaseTransitionC;

    private static PhaseTransition.Phase moveArmToSpecifiedPosition(double targetHeight, String phaseName) {
        return new PhaseTransition.Phase(
                () -> {
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = targetHeight;
                    State.Arm.targetDepth = State.armToTag;
                },
                (double time) -> {
                    return State.Arm.isArmAtTarget;
                },
				() -> {
					State.Arm.resetPidController = true;
				},
				phaseName
		);
	}
	public static PhaseTransition.Phase releaseHand (String phaseName){
		return new PhaseTransition.Phase(
				() -> {
					State.Hand.grabHandState = GrabHandState.s_releaseHand;
					return;
				},
				(double time) -> {
					return State.Hand.isRelease;
				},
				phaseName
		);
	}

	public static void autonomousInit () {
		phaseTransitionA.registerPhase(
				moveArmToSpecifiedPosition(Const.Calculation.Camera.GoalHeight - Const.Arm.RootHeight, "move arm to cube goal"),
				releaseHand("release cube")
		);

		phaseTransitionB.registerPhase(
				moveArmToSpecifiedPosition(Const.Calculation.Limelight.GoalHeight - Const.Arm.RootHeight, "move arm to corn goal"),
				releaseHand("release corn")
		);
	}

	public static void run () {
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
				System.out.println("やばいよ");
				break;
		}
	}
}
