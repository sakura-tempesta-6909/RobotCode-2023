package frc.robot.subClass;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.State;

public class ExternalSensors {
    PowerDistribution mainPDP;
    public ExternalSensors() {
        CameraServer.startAutomaticCapture();
        mainPDP = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
    }

    /**
     * 外部のセンサーの値を読む。
     */
    public void readExternalSensors(){
    }
}