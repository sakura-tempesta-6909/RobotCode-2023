package frc.robot.mode;

import frc.robot.State;

public class ChargeStationMode extends Mode{
// 正式名称は「ワクワクドキドキ神様お願いブレイクモード」
    @Override
    public void changeMode() {

    }

    @Override
    public void changeState() {
        State.Drive.isMotorBrake = true;
    }
}
