package frc.robot.Mode;

import frc.robot.State;
import frc.robot.SubClass.Const;

import java.awt.*;

public class ArmMode extends Mode{

    @Override
    public void changeMode() {

    }

    /**
     * XButton -> axisControlMode(ターゲット座標をコントローラーで変える)
     * AButton -> autoControlMode(limelightで特定された座標へ)
     * NoButton -> rotationControlMode(各アームの角度をコントローラーで変える -> もっとも直感的)
     * */
    @Override
    public void changeState() {

        if (driveController.getXButtonPressed()){
            resetArmTargets();
            State.armState = State.ArmState.s_axisCtrl;
        } else if (driveController.getAButtonPressed()) {
            resetArmTargets();
            State.armState = State.ArmState.s_axisCtrl;
        } else {
            State.armState = State.ArmState.s_rotationCtrl;
        }

        if (driveController.getXButton()){
            // Targetの座標ををコントローラーによって変える
            State.armTargetX += driveController.getRightX() * Const.Arms.TargetModifyRatio;
            State.armTargetZ += driveController.getRightY() * Const.Arms.TargetModifyRatio;
        } else if (driveController.getAButton()) {
            //TODO ここでlimelightの値を代入
            State.armTargetX = 10;
            State.armTargetZ = 10;
        }
    }

    private void resetArmTargets () {
        State.armTargetX = State.armAxisX;
        State.armTargetZ = State.armAxisZ;
    }
}
