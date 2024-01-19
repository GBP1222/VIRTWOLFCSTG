package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends OpMode
{

    private  Servo
            ServoStanga,  ServoDreapta;
    public static int targetstanga = 0;
    public static int targetdreapta = 0;
    @Override
    public void init()
    {


        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

    }

    @Override
    public void loop()
    {
        ServoDreapta.setPosition(targetdreapta);
//            ServoStanga.setPosition(targetstanga);
//            ServoDreapta.setPosition(targetdreapta);


        //telemetry.addData("PosServoStanga ", XXX);
        //telemetry.addData("PosServoDreapta", XXX);
        telemetry.update();


    }
}
