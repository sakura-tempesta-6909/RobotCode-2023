package frc.robot.mode;

import frc.robot.States.State;

public class ChargeStationMode extends Mode{
// 正式名称は「ワクワクドキドキ神様お願いブレイクモード」
    @Override
    public void changeMode() {
        if (driveController.getStartButtonPressed()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getBackButtonPressed()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
        }
    }

    @Override
    public void changeState() {
        State.Drive.isMotorBrake = true;
        State.Drive.xSpeed = -1 * driveController.getLeftY();
        State.Drive.zRotation = -1 * driveController.getRightX();

        State.Drive.state = State.Drive.States.s_fastDrive;
    }
}
