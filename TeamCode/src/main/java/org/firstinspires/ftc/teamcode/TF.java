package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

@TeleOp(name = "TensorFlow movement")
public class TF extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    private TrajectorySequence Incercare1;

    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/modelCoroana.tflite";
    private static final String TFOD_MODEL_FILE2 = "/sdcard/FIRST/tflitemodels/modelCoroanaRosu.tflite";
    private static final String[] LABELS = {
            "con",
    };

    TfodProcessor tfod;
    VisionPortal visionPortal;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-33.00, -64.00, Math.toRadians(90));

        ElapsedTime timer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
//new Pose2d(-36.18, -66.72, Math.toRadians(89.73))
        TrajectorySequence Incercare1 = drive.trajectorySequenceBuilder(startPose)
                .forward(1) // Adjust the distance as needed
//                .addTemporalMarker(0.1, () -> {
//                })
                .waitSeconds(10)
                .strafeRight(1)
                .build();

        initTfod();

        telemetry.addData("Previzualizare DS on/off", "3 puncte, Fluxul camerei");
        telemetry.addData(">", "Atingeți Play pentru a începe OpMode");
        telemetry.update();
//        waitForStart();

            while (opModeIsActive()){
                telemetryTfod();
                processConePosition();
                telemetry.update();
                drive.update();
                sleep(20);
            }

        visionPortal.close();
    }
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.75f);
        visionPortal.setProcessorEnabled(tfod, true);
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Obiecte Detectate", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Imagine", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Poziție", "%.0f / %.0f", x, y);
            telemetry.addData("- Dimensiune", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }

    private void processConePosition() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions.isEmpty()) {
            // No recognitions found, handle accordingly or return
            return;
        }

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            double relativePosition = x - 320;

            String conePosition = determineConePosition(relativePosition);

            telemetry.addData("Poziția Conului", conePosition);

            planTrajectory(conePosition);

            drive.update();
        }
    }

    private String determineConePosition(double relativePosition) {
        double LEFT_STRIP_POSITION = -100;
        double RIGHT_STRIP_POSITION = 100;

        if (relativePosition < LEFT_STRIP_POSITION) {
            return "Stanga";
        } else if (LEFT_STRIP_POSITION <= relativePosition && relativePosition <= RIGHT_STRIP_POSITION) {
            return "Mijloc";
        } else {
            return "Dreapta";
        }
    }

    private Trajectory trajectory;

    private void planTrajectory(String conePosition) {
        telemetry.addData("PING", runtime.seconds());
//
//        Pose2d startPose = drive.getPoseEstimate();
//        TrajectoryBuilder builder = drive.trajectoryBuilder(startPose);
//
//        if (conePosition.equals("Stanga")) {
//            builder.splineTo(new Vector2d(-36.18, -33.98), Math.toRadians(90.00));
//        } else if (conePosition.equals("Dreapta")) {
//            builder.splineTo(new Vector2d(-36.18, -33.98), Math.toRadians(90.00));
//        } else {
//            builder.splineTo(new Vector2d(-36.18, -33.98), Math.toRadians(90.00));
//        }
//
//        trajectory = builder.build();
//        drive.followTrajectory(trajectory);
//
//        drive.update();

//        Pose2d startPose = drive.getPoseEstimate();

        // Use drive.trajectoryBuilder for all cases
//        TrajectorySequence autonom = drive.trajectoryBuilder(startPose)
//                .forward(1) // Adjust the distance as needed
//                .addTemporalMarker(0.1, () -> {
//                })
//                .waitSeconds(10)
//                .strafeRight(1)
//                .build();

        // Follow the trajectory
//        drive.followTrajectoryAsync(Incercare1);
//
//        drive.update();
    }
}
// Paralel = 0
// Tabla = 1
// Posibil init = 0.5