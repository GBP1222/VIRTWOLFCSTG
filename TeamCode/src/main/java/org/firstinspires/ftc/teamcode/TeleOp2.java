package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2TELEOP")
public class TeleOp2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotHardware2 robot = new RobotHardware2(hardwareMap);
        robot.ClawLeftOpen();
        robot.ClawRightOpen();

        runtime.reset();
        waitForStart();
        while (opModeIsActive())
        {
            robot.BratPID(gamepad2);
            robot.LiftPID(gamepad2);
            robot.DriveMovement(gamepad1);
            robot.InclineClawManager(gamepad2);
            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.addData("Brat Pos ", robot.Brat.getCurrentPosition());

            telemetry.update();

        }
    }

}