package frc.robot.mode;
import frc.robot.State;

public class TestMode extends Mode {
    @Override
    public void changeMode() {
    }

    @Override
    public void changeState() {
        if (driveController.getAButtonPressed()) {
            State.Drive.resetPosition = true;
        }
        if (driveController.getAButton()) {
            State.Drive.targetMeter = 1;
            State.Drive.state = State.Drive.States.s_pidDrive;
        } else {
            State.Drive.state = State.Drive.States.s_stopDrive;
        }
    }
}
