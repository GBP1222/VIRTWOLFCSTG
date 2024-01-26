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

// AUTONOM ALBASTRU LUNG TABLOU IN STANGA
@TeleOp(name = "AUTONOM ALBASTRU LUNG")
public class AlbastruLung extends LinearOpMode {
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

        drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robot = new RobotHardware(hardwareMap);

        initTfod();
        telemetryTfod();

        Pose2d startPose = new Pose2d(-32.78, 64.61, Math.toRadians(268.51)); // determina pozitia reala de start

        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) auto_case = processConePosition();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(-33.48, 31.74), Math.toRadians(-1.47))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                                                          // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(-19.16, 34.26), Math.toRadians(0.00))
                .splineTo(new Vector2d(47.81, 34.84), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                                                            // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(48.00, 31.55), Math.toRadians(270.00))
                .splineTo(new Vector2d(48.39, 11.81), Math.toRadians(-88.59))
                .splineTo(new Vector2d(50.71, 11.81), Math.toRadians(0.00))
                .splineTo(new Vector2d(62.52, 12.19), Math.toRadians(0.00))
                //parcare
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(-36.19, 33.10), Math.toRadians(270.00))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(-32.90, 32.90), Math.toRadians(-2.16))
                .splineTo(new Vector2d(49.55, 33.87), Math.toRadians(0.67))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(49.55, 30.97), Math.toRadians(270.00))
                .splineTo(new Vector2d(48.58, 13.16), Math.toRadians(266.89))
                .splineTo(new Vector2d(51.10, 12.58), Math.toRadians(0.00))
                .splineTo(new Vector2d(61.94, 12.39), Math.toRadians(-1.02))
                //parcare
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(-47.94, 41.49), Math.toRadians(-84.05))
                //lasa pixelu prima banda
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(-14.32, 34.45), Math.toRadians(0.00))
                .splineTo(new Vector2d(48.39, 34.45), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(48.58, 31.55), Math.toRadians(-87.74))
                .splineTo(new Vector2d(49.16, 12.58), Math.toRadians(-88.25))
                .splineTo(new Vector2d(51.48, 12.39), Math.toRadians(-4.76))
                .splineTo(new Vector2d(61.35, 12.39), Math.toRadians(0.00))
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