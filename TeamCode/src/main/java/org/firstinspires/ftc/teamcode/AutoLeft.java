package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="RoadRunnerStanga")
public class AutoLeft extends LinearOpMode
{

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(-33.00, -64.00, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        //region AprilTag

        telemetry.setMsTransmissionInterval(50);

        TrajectorySequence parcare1 = drive.trajectorySequenceBuilder(startPose)
                //preload
                .forward(8)
                .waitSeconds(6)
                .turn(Math.toRadians(-180.00))
                .waitSeconds(6)
                .forward(100)

                .build();

            drive.followTrajectorySequenceAsync(parcare1);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            telemetry.update();
        }
    }



}