package org.firstinspires.ftc.teamcode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_Glisiera extends OpMode
{
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private final double ticks_in_degrees = 700/180.0; //idk daca e val corecta we'll have to see abt it;
    private DcMotorEx
                LiftStanga, LiftDreapta;

    @Override
    public void init()
    {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta)");


    }

    @Override
    public void loop()
    {
        controller.setPID(p,i,d);
        int armPos = LiftStanga.getCurrentPosition();
                    //sau LiftDreapta.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        LiftDreapta.setPower(power);
        LiftStanga.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", armPos);
        telemetry.update();


    }
}
