package frc.robot.mode;

import frc.robot.States.State;

public class TestMode extends Mode{
    @Override
    public void changeMode() {}
    
    @Override
    public void changeState() {

        if(driveController.getXButton()){
            State.Drive.state=State.Drive.States.s_pidDrive;
            State.Drive.targetMeter = State.Drive.leftMeter;
        }
    }
}
