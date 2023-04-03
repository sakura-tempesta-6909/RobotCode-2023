package frc.robot.mode;

import frc.robot.states.DriveState;

public class TestMode extends Mode{
    @Override
    public void changeMode() {}
    
    @Override
    public void changeState() {

        if(driveController.getXButton()){
            DriveState.driveState =DriveState.DriveStates.s_pidDrive;
            DriveState.targetMeter = DriveState.leftMeter;
        }
    }
}
