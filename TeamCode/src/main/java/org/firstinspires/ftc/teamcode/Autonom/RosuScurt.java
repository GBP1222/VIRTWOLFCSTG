package org.firstinspires.ftc.teamcode.Autonom;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

// AUTONOM ROSU SCURT TABLOU IN DREAPTA
@Autonomous(name = "AUTONOM ROSU SCURT")
public class RosuScurt extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(7.20, -66.32, Math.toRadians(90.0)); // determina pozitia reala de start

        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) auto_case = processConePosition();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(4.74, -40.17), Math.toRadians(122.28))
                //lasa pixelu prima banda
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(7.20, -39.03), Math.toRadians(47.29))
                .splineTo(new Vector2d(49.64, -35.43), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(49.26, -38.08), Math.toRadians(265.60))
                .splineTo(new Vector2d(48.32, -61.77), Math.toRadians(267.71))
                .splineTo(new Vector2d(50.59, -61.58), Math.toRadians(-6.01))
                .splineTo(new Vector2d(61.77, -61.58), Math.toRadians(-1.85))
                //parcare
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(11.56, -34.48), Math.toRadians(88.36))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(13.83, -34.48), Math.toRadians(1.02))
                .splineTo(new Vector2d(50.78, -34.86), Math.toRadians(-0.59))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(50.59, -37.52), Math.toRadians(261.03))
                .splineTo(new Vector2d(48.32, -62.34), Math.toRadians(-88.69))
                .splineTo(new Vector2d(50.59, -62.34), Math.toRadians(0.00))
                .splineTo(new Vector2d(61.96, -62.34), Math.toRadians(0.00))
                //parcare
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)// adauga pasi aici
                .splineTo(new Vector2d(22.17, -41.31), Math.toRadians(83.88))
                //lasa pixelu
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                .splineTo(new Vector2d(22.74, -40.74), Math.toRadians(3.81))
                .splineTo(new Vector2d(50.21, -38.84), Math.toRadians(0.00))
                //lasa pixelu pe tablou
                //muta bratu de tot in fata
                .addTemporalMarker(() -> robot.OpenClaw())// schimba asta la toate /
                // trebuie functie pt fiecare parte din gheara :0
                .waitSeconds(0.01)
                .addTemporalMarker(() -> robot.CloseClaw())
                //muta bratu inapoi
                .splineTo(new Vector2d(50.02, -41.87), Math.toRadians(265.24))
                .splineTo(new Vector2d(48.51, -60.63), Math.toRadians(265.38))
                .splineTo(new Vector2d(51.35, -60.44), Math.toRadians(3.81))
                .splineTo(new Vector2d(62.53, -61.01), Math.toRadians(0.00))
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
                .setModelFileName(TFOD_MODEL_FILE2)
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