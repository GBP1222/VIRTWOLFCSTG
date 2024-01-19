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
    public static double targetstanga = 0;
    public static double targetdreapta = 0;
    @Override
    public void init()
    {


        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

        ServoStanga.setDirection(Servo.Direction.FORWARD);
        ServoDreapta.setDirection(Servo.Direction.FORWARD);


        //FRONT - STANGA REVERSE / DREAPTA FORWARD
        //BACK  - STANGA FORWARD /  DREAPTA REVERSE
    }

    @Override
    public void loop()
    {
        if(gamepad1.a) {
            ServoStanga.setPosition(0);
            ServoDreapta.setPosition(1);
        }
        if(gamepad1.b) {
            ServoStanga.setPosition(1);
            ServoDreapta.setPosition(0);
        }


//            ServoStanga.setPosition(targetstanga);
//            ServoDreapta.setPosition(targetdreapta);


        //telemetry.addData("PosServoStanga ", XXX);
        //telemetry.addData("PosServoDreapta", XXX);
        telemetry.update();


    }
}
