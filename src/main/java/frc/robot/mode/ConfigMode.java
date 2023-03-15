package frc.robot.mode;

import frc.robot.State;

public class ConfigMode extends Mode{

    @Override
    public void changeMode() {
        if (driveController.getStartButtonPressed()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getBackButtonPressed()) {
            State.mode = State.Modes.k_arm;
        }
    }

    @Override
    public void changeState() {
        if (driveController.getXButtonPressed()) {
            State.Arm.resetEncoder = true;
        }

        if (driveController.getYButton()) {
            State.isCompressorEnable = false;
        }
    }
}
