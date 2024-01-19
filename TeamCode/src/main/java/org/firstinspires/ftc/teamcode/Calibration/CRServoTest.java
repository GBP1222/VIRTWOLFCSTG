package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class CRServoTest extends OpMode
{

    private  CRServo
            ServoStanga, ServoDreapta;
    public static double targetstanga = -0.1;
    public static double targetdreapta = 0;
    @Override
    public void init()
    {


        ServoStanga = hardwareMap.get(CRServo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(CRServo.class, "ServoDreapta");
        //FRONT - STANGA REVERSE / DREAPTA FORWARD
        //BACK  - STANGA FORWARD /  DREAPTA REVERSE
    }

    @Override
    public void loop()
    {
        if(gamepad1.a) {
            ServoStanga.setPower(targetstanga);
            ServoStanga.setDirection(CRServo.Direction.REVERSE);
            ServoDreapta.setDirection(CRServo.Direction.FORWARD);
            ServoStanga.setPower(targetstanga);
            ServoDreapta.setPower(targetdreapta);
        }
        else if(gamepad1.b) {
            ServoStanga.setPower(targetstanga);
            ServoStanga.setDirection(CRServo.Direction.FORWARD);
            ServoDreapta.setDirection(CRServo.Direction.REVERSE);
            ServoStanga.setPower(targetstanga);
            ServoDreapta.setPower(targetdreapta);

        }else {
            ServoStanga.setPower(0);
            ServoDreapta.setPower(0);
        }


        telemetry.addData("Poz", ServoStanga.getPower());
//            ServoStanga.setPosition(targetstanga);
//            ServoDreapta.setPosition(targetdreapta);


        //telemetry.addData("PosServoStanga ", XXX);
        //telemetry.addData("PosServoDreapta", XXX);
        telemetry.update();


    }
}
