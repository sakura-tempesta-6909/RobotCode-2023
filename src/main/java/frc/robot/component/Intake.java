package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.states.IntakeState;
import frc.robot.states.State;
import frc.robot.consts.DriveConst;
import frc.robot.consts.IntakeConst;

public class Intake implements Component{
    private final Solenoid intakeSolenoid;
    private final TalonSRX rightRoller;
    private final TalonSRX leftRoller;
    private final VictorSPX bottomRoller;
    private final Compressor compressor;

    public Intake() {
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConst.Ports.IntakeSolenoid);
        rightRoller = new TalonSRX(IntakeConst.Ports.RightRoller);
        leftRoller = new TalonSRX(IntakeConst.Ports.LeftRoller);
        bottomRoller = new VictorSPX(IntakeConst.Ports.BottomRoller);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);

       bottomRoller.setInverted(false);
    }
        
    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        
    }

    /** Roller関係のモーターを動かす */
    public void rollerControl(double sideRollerSpeed, double bottomRollerSpeed) {
       rightRoller.set(ControlMode.PercentOutput, sideRollerSpeed);
       leftRoller.set(ControlMode.PercentOutput, sideRollerSpeed);
       bottomRoller.set(ControlMode.PercentOutput, bottomRollerSpeed);

   }

    /** Intake関係のsolenoidを動かす */
    public void intakeControl(boolean isExtendingIntake) {
        intakeSolenoid.set(isExtendingIntake);
    }



    // /** CONEとCUBEを出す */
    public void outtakeGamePiece() {
         rollerControl(IntakeConst.Speeds.SideRollerOuttakeSpeed, IntakeConst.Speeds.BottomRollerOuttakeSpeed);
    }

     /** CONEとCUBEを回収する */
     public void intakeGamePiece() {
         rollerControl(IntakeConst.Speeds.SideRollerIntakeSpeed, IntakeConst.Speeds.BottomRollerIntakeSpeed);
     }

     /** Conveyorを動かさない */
     public void stopRoller() {
         rollerControl(DriveConst.Speeds.Neutral, DriveConst.Speeds.Neutral);
     }


    /** Intakeを出す */
    public void openIntake() {
        intakeControl(true);
    }

    /** Rollerをしまう */
    public void closeIntake() {
        intakeControl(false);
    }

    @Override
    public void applyState() {
        if (IntakeState.isCompressorEnable) {
            compressor.enableDigital();
        } else {
            compressor.disable();
        }

        if (IntakeState.intakeExtensionState == IntakeState.IntakeExtensionStates.s_closeIntake) {
            stopRoller();
        } else {


        switch(IntakeState.intakeState){
            case s_outtakeGamePiece:
                 outtakeGamePiece();
                 break;
            case s_intakeGamePiece:
                 intakeGamePiece();
                 break;
            case s_stopRoller:
                 stopRoller();
                 break;
        }
    }
        
        switch(IntakeState.intakeExtensionState){
            case s_openIntake:
                openIntake();
                break;
            case s_closeIntake:
                closeIntake();
                break;
        }
    }
}