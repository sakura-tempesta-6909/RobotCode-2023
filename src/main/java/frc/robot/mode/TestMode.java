package frc.robot.mode;

import edu.wpi.first.math.util.Units;
import frc.robot.State;
import org.opencv.calib3d.StereoBM;

public class TestMode extends Mode{
    @Override
    public void changeMode() {}
    
    @Override
    public void changeState() {
        if(driveController.getAButtonPressed()){
            State.Drive.resetPIDController = true;
        }
        if(driveController.getAButton()){
            State.Drive.targetMeter = 1;
        }
    }
}
