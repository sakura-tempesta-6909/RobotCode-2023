package frc.robot.mode;

import frc.robot.State;

public class DriveMode extends Mode {

    @Override
    public void changeMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void changeState() {
        State.driveXSpeed = driveController.getLeftY();
        State.driveZRotation = driveController.getRightX();
        
    }
    
}