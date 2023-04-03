package frc.robot.mode;

import frc.robot.states.ArmState;
import frc.robot.states.State;
import frc.robot.consts.ArmConst;
import frc.robot.consts.LimelightConst;

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
            ArmState.targetHeight = LimelightConst.TopGoalHeight - ArmConst.RootHeightFromGr;
            ArmState.targetDepth = ArmState.TargetDepth.TopCorn;
        }
        
    }
    
}
