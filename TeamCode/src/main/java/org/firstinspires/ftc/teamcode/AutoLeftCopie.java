//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name="RRCOPIESTANGA")
//public class AutoLeftCopie extends LinearOpMode
//{
//    private DcMotor leftFront;
//    private DcMotor leftRear;
//    private DcMotor rightFront;
//    private DcMotor rightRear;
//
//    @Override
//    public void runOpMode()
//    {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Pose2d startPose = new Pose2d(-33.00, -64.00, Math.toRadians(90));
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setPoseEstimate(startPose);
//
//        //region AprilTag
//
//        telemetry.setMsTransmissionInterval(50);
//
//                //preload
//        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-41.32, -64.37, Math.toRadians(90.00)))
//                .splineTo(new Vector2d(-36.18, -34.72), Math.toRadians(90.00))
//                .splineTo(new Vector2d(-32.95, -18.13), Math.toRadians(48.61))
//                .splineTo(new Vector2d(6.39, -11.23), Math.toRadians(4.64))
//                .splineTo(new Vector2d(25.91, -13.72), Math.toRadians(-21.80))
//                .splineTo(new Vector2d(48.22, -35.60), Math.toRadians(-2.05))
//                .splineTo(new Vector2d(35.45, -36.33), Math.toRadians(190.62))
//                .splineTo(new Vector2d(60.99, -11.38), Math.toRadians(0.00))
//                .build();
//
//        drive.setPoseEstimate(untitled0.start());
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//            telemetry.update();
//        }
//    }
//
//
//
//}