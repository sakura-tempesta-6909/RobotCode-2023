package frc.robot.mode;

import frc.robot.State;
import frc.robot.State.DriveState;

public class DriveMode extends Mode {

    @Override
    public void changeMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void changeState() {
        State.driveXSpeed = driveController.getLeftY();
        State.driveZRotation = driveController.getRightX();

        if (driveController.getAButton()) {
            State.driveState = DriveState.s_apriltagTracking;
        } else {
            State.driveState = DriveState.s_stopDrive;
        }

        if (driveController.getBButton()) {
            State.driveState = DriveState.s_targetTracking;
        } else {
            State.driveState = DriveState.s_stopDrive;
        }
    }
    
}
