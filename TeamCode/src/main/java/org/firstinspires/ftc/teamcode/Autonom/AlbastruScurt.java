package org.firstinspires.ftc.teamcode.Autonom;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

// AUTONOM ALBASTRU SCURT TABLOU IN STANGA
@TeleOp(name = "AUTONOM ALBASTRU SCURT")
public class AlbastruScurt extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/modelCoroana.tflite";
    private static final String TFOD_MODEL_FILE2 = "/sdcard/FIRST/tflitemodels/modelCoroanaRosu.tflite";
    private static final String[] LABELS = {
            "con",
    };

    int auto_case = 2;
    TfodProcessor tfod;
    VisionPortal visionPortal;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        initTfod();
        telemetryTfod();

        Pose2d startPose = new Pose2d(15.29, 62.52, Math.toRadians(270.00)); // determina pozitia reala de start

        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) auto_case = processConePosition();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(23.42, 39.68), Math.toRadians(270.00))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(24.00, 39.48), Math.toRadians(0.00))
                .splineTo(new Vector2d(49.35, 35.23), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(49.35, 35.03), Math.toRadians(88.53))
                .splineTo(new Vector2d(49.94, 59.42), Math.toRadians(88.75))
                .splineTo(new Vector2d(49.94, 59.23), Math.toRadians(2.12))
                .splineTo(new Vector2d(61.74, 60.00), Math.toRadians(0.00))
                //parcare
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(12.39, 32.71), Math.toRadians(270.00))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(13.16, 32.90), Math.toRadians(-2.20))
                .splineTo(new Vector2d(50.71, 33.29), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(50.32, 33.68), Math.toRadians(91.68))
                .splineTo(new Vector2d(49.94, 58.84), Math.toRadians(90.88))
                .splineTo(new Vector2d(49.74, 58.84), Math.toRadians(0.00))
                .splineTo(new Vector2d(62.71, 59.23), Math.toRadians(0.00))
                //parcare
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(10.26, 33.29), Math.toRadians(180.00))
                //lasa pixelu prima banda
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(10.45, 33.29), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.52, 33.87), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(50.71, 34.26), Math.toRadians(90.00))
                .splineTo(new Vector2d(50.13, 57.68), Math.toRadians(91.42))
                .splineTo(new Vector2d(50.71, 57.48), Math.toRadians(5.71))
                .splineTo(new Vector2d(62.90, 59.03), Math.toRadians(1.85))
                //parcare
                .build();

        if(auto_case==1)
            drive.followTrajectorySequenceAsync(left);
        else if(auto_case==2)
            drive.followTrajectorySequenceAsync(middle);
        else if(auto_case==3)
            drive.followTrajectorySequenceAsync(right);


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
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

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

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

    private int processConePosition(){
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        String conePosition = "";
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double relativePosition = x - 320;

            conePosition = determineConePosition(relativePosition);
        }
        if(conePosition == "Stanga"){
            return 1;
        }else if (conePosition == "Dreapta"){
            return 3;
        }
        return 2;
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
}