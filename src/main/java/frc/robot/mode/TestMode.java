package frc.robot.mode;

import frc.robot.State;
import frc.robot.State.Arm.States;

public class TestMode extends Mode{
    @Override
    public void changeMode() {}
    
    @Override
    public void changeState() {

        if(driveController.getXButton()){
            State.Drive.state=State.Drive.States.s_pidDrive;
            State.Drive.targetLength = State.Drive.leftLength;
        }
    }
}
