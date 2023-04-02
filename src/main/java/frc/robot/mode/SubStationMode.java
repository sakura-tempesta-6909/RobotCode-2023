package frc.robot.mode;

import frc.robot.States.State;
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
            State.Arm.targetHeight = LimelightConst.TopGoalHeight - ArmConst.RootHeightFromGr;
            State.Arm.targetDepth = State.Arm.TargetDepth.TopCorn;
        }
        
    }
    
}
