package frc.robot.consts;

public class MQTTConst {
    public static final String Broker = "tcp://raspberrypi.local:1883";
    public static final String Topic = "robot/data/main";
    public static final int MaxRetry = 100;
    public static final String ClientId = "robot/test";

    public static void MQTTConstInit() {

    }
}
