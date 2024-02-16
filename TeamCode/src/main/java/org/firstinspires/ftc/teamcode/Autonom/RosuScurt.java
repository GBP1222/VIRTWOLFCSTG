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

    int auto_case;
    TfodProcessor tfod;
    VisionPortal visionPortal;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robot = new RobotHardware(hardwareMap);

        initTfod();
        telemetryTfod();

        Pose2d startPose = new Pose2d(15.05, -62.75, Math.toRadians(90.00)); // determina pozitia reala de start

        drive.setPoseEstimate(startPose);

        robot.setLiftTarget(0);
//        robot.setBratTarget(0);
        robot.RetractClaw();
        robot.OpenClaw();
        robot.InitDrone();

        while (!isStarted() && !isStopRequested()) auto_case = processConePosition();

        telemetry.addData("Caz", auto_case);
        telemetry.update();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(15.05, -62.75, Math.toRadians(90.00)))
                .splineTo(new Vector2d(16.07, -48.37), Math.toRadians(95.90))
                .lineToLinearHeading(new Pose2d(14.61, -33.25, Math.toRadians(0.00)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> robot.ParallelClaw())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.ClawLeftOpen())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.RetractClaw())
                .waitSeconds(0.2)
                //.splineTo(new Vector2d(43.38, -42.79), Math.toRadians(0.00))
                .lineTo(new Vector2d(43.08, -59.38))
                .lineToLinearHeading(new Pose2d(60.55, -59.08, Math.toRadians(90.00)))
                .addTemporalMarker(() -> robot.ClawRightOpen())
//                .lineTo(new Vector2d(62.75, -25.03))

                // .splineTo(new Vector2d(49.39, -35.89), Math.toRadians(0.00)) //.splineTo(new Vector2d(50.28, -40.29), Math.toRadians(0.00))

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(15.05, -62.75, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(14.02, -13.72), Math.toRadians(88.09))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> robot.ParallelClaw())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.ClawLeftOpen())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.RetractClaw())
                .waitSeconds(0.2)
                .splineTo(new Vector2d(31.63, -14.31), Math.toRadians(-40.45))
                //.splineTo(new Vector2d(43.38, -42.79), Math.toRadians(0.00))
                .lineTo(new Vector2d(42.06, -59.96))
                .lineToLinearHeading(new Pose2d(59.82, -58.94, Math.toRadians(90.00)))
                .addTemporalMarker(() -> robot.ClawRightOpen())

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(15.05, -62.75, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(23.85, -20.18), Math.toRadians(88.09))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> robot.ParallelClaw())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.ClawLeftOpen())
                .waitSeconds(1.2)
                .addTemporalMarker(() -> robot.RetractClaw())
                .waitSeconds(0.2)
                .splineTo(new Vector2d(34.86, -12.99), Math.toRadians(-46.65))
                .splineTo(new Vector2d(43.38, -42.79), Math.toRadians(0.00))
                //.lineTo(new Vector2d(42.64, -59.38))
                .lineToLinearHeading(new Pose2d(61.43, -60.11, Math.toRadians(90.00)))
                .addTemporalMarker(() -> robot.ClawRightOpen())
                .build();


        if(auto_case==1)
            drive.followTrajectorySequenceAsync(left);
        else if(auto_case==2)
            drive.followTrajectorySequenceAsync(middle);
        else if(auto_case==3)
            drive.followTrajectorySequenceAsync(right);


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Caz", auto_case);
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
            telemetry.addData("Poziția Conului", conePosition);
        }
        if(conePosition == "Stanga"){
            return 1;
        }else if (conePosition == "Dreapta"){
            return 3;
        }
        else
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