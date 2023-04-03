package frc.robot.mode;

import frc.robot.states.ArmState;
import frc.robot.states.IntakeState;
import frc.robot.states.State;

public class ConfigMode extends Mode{

    @Override
    public void changeMode() {
        if (driveController.getStartButtonPressed()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getBackButtonPressed()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        }
    }

    @Override
    public void changeState() {
        if (driveController.getXButtonPressed()) {
            ArmState.resetEncoder = true;
        }

        if (driveController.getYButton()) {
            IntakeState.isCompressorEnable = false;
        } else if (driveController.getAButton()) {
            IntakeState.isCompressorEnable = true;
        }

        if (driveController.getAButton()) {
            ArmState.isMoveLeftAndRightEncoderReset = true;
        }
    }
}
