package frc.robot.mode;

import frc.robot.states.ArmState;
import frc.robot.states.State;
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
            ArmState.targetHeight = Const.Calculation.Limelight.TopGoalHeight - Const.Arm.RootHeightFromGr;
            ArmState.targetDepth = ArmState.TargetDepth.TopCorn;
        }
        
    }
    
}
