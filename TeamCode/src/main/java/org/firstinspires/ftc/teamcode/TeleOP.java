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
        robot.setBratTarget(0);
        robot.RetractClaw();
        robot.InitDrone();
        robot.OpenClaw();
//        robot.InitBratLift();

        runtime.reset();
        waitForStart();
        while (opModeIsActive())
        {

            robot.LiftPID(gamepad2);
            robot.BratPID(gamepad2);
            robot.ClawManager(gamepad2);
            robot.InclineClawManager(gamepad2);
            robot.DriveMovement(gamepad1);
            robot.DroneManager(gamepad1);


            telemetry.addData("Lift Position", robot.getLiftLeftPosition());
//            telemetry.addData("Lift Position Left", robot.getLiftLeftPosition());
            telemetry.addData("Lift Target", robot.getLiftTarget());
            telemetry.addData("Lift Power", robot.getLiftPower());

            telemetry.addData("Arm Position", robot.getArmPosition());
//            telemetry.addData("Lift Position Left", robot.getLiftLeftPosition());
            telemetry.addData("Arm Target", robot.getArmTarget());
            telemetry.addData("Arm Power", robot.getLiftPower());
            
            

            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.update();

        }
    }

}