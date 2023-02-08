package frc.robot.phase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.function.DoublePredicate;

import edu.wpi.first.wpilibj.Timer;

public class PhaseTransition {
	private ArrayList<Phase> phaseList = new ArrayList<>();

	private Timer timer = new Timer();

	PhaseTransition() {
		timer.reset();
		timer.start();
	}

	private Phase currentPhase;
	private Iterator<Phase> phaseIterator;
	private boolean is_finished = false;

	public void registerPhase(Phase... phases) {
		if(phases.length == 0) {
			return;
		}

		Collections.addAll(phaseList, phases);

		phaseIterator = phaseList.iterator();
		currentPhase = phaseIterator.next();
	}

	public void run() {
		if(is_finished) {
			System.out.println("All phases is finished.");
			return;
		}

		if(currentPhase.condition.test(timer.get())) {
			currentPhase.onSuccess.run();
			
			timer.reset();
			timer.start();
			System.out.println(currentPhase.toString() + " has been finished.");

			if(phaseIterator.hasNext()) {
				currentPhase = phaseIterator.next();
			} else {
				System.out.println("All phases have been finished!!");
				is_finished = true;
			}
		} else {
			currentPhase.action.run();
		}
	}


	public static class Phase {
		Runnable action;
		DoublePredicate condition;
		Runnable onSuccess;

		private final int phaseID;
		private final String phaseName;

		private static int phaseCounter = 0;
		public static void PhaseInit() {
			phaseCounter = 0;
		}

		public Phase(Runnable action, DoublePredicate condition) {
			this(action, condition, () -> {}, "anonymous phase");
		}

		public Phase(Runnable action, DoublePredicate condition, String phaseName) {
			this(action, condition, () -> {}, phaseName);
		}

		public Phase(Runnable action, DoublePredicate condition, Runnable onSuccess, String phaseName) {
			this.action = action;
			this.condition = condition;
			this.onSuccess = onSuccess;
			this.phaseID = phaseCounter++;
			this.phaseName = phaseName;
		}

		public String toString() {
			return "phase" + this.phaseID + " (" + phaseName + ")";
		}
	}
}