package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="1TELEOP")
public class TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.setLiftTarget(0);
        robot.InitBratLift();

        runtime.reset();
        waitForStart();
        while (opModeIsActive())
        {

            robot.LiftPID(gamepad2);
            robot.BratPID(gamepad2);
            robot.DriveMovement(gamepad1);
            robot.ClawManager(gamepad2);


            telemetry.addData("Lift Position", robot.getLiftLeftPosition());
//            telemetry.addData("Lift Position Left", robot.getLiftLeftPosition());
            telemetry.addData("Lift Target", robot.getLiftTarget());
            telemetry.addData("Lift Power", robot.getLiftPower());

            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.update();

        }
    }

}