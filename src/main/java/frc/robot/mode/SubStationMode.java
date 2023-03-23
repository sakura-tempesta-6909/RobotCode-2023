package frc.robot.mode;

import frc.robot.States.State;
import frc.robot.subClass.Const;

public class SubStationMode extends Mode {

    @Override
    public void changeMode() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void changeState() {
        // TODO Auto-generated method stub
        if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            State.Arm.targetHeight = Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
            State.Arm.targetDepth = State.Arm.TargetDepth.TopCorn;
        }
        
    }
    
}
