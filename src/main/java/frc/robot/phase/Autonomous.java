package frc.robot.phase;

import frc.robot.State;
import frc.robot.State.DriveState;
import frc.robot.State.GrabHandState;
import edu.wpi.first.math.util.Units;

public class Autonomous {
	private static PhaseTransition phaseTransitionA;
	private static PhaseTransition phaseTransitionB;
	private static PhaseTransition phaseTransitionC;

	public static PhaseTransition.Phase releaseHand(String phaseName) {
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

	public static void autonomousInit() {

	}

	public static void run() {

	}
}
