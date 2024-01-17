package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name = "TensorFlow cu movement")
public class TFMiscare extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    // Calea către modelul TensorFlow și etichetele claselor
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/modelCoroana.tflite";
    private static final String[] LABELS = {
            "con",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        initTfod();

        telemetry.addData("Previzualizare DS on/off", "3 puncte, Fluxul camerei");
        telemetry.addData(">", "Atingeți Play pentru a începe OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();
                processConePosition();
                telemetry.update();

                // Economisiți resurse CPU; se poate relua streaming-ul când este necesar.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                sleep(20);
            }
        }

        visionPortal.close();
    }

    /**
     * Inițializați procesorul de detecție a obiectelor TensorFlow.
     */
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Creați portalul de viziune utilizând un constructor.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Setează camera (webcam sau camera RC încorporată).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Alegeți o rezoluție a camerei. Nu toate camerele acceptă toate rezoluțiile.
        builder.setCameraResolution(new Size(640, 480));

        // Activare previzualizare RC (LiveView). Setează "false" pentru a omite monitorizarea camerei.
        builder.enableLiveView(true);

        // Setează formatul fluxului; MJPEG consumă mai puțină lățime de bandă decât YUY2 implicit.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Alegeți dacă LiveView se oprește sau nu dacă nu există procesori activați.
        // Dacă setat "true", monitorul afișează un ecran portocaliu solid dacă nu există procesori activați.
        // Dacă setat "false", monitorul afișează imaginea camerei fără adnotări.
        builder.setAutoStopLiveView(false);

        // Setează și activează procesorul.
        builder.addProcessor(tfod);

        // Construiește Vision Portal folosind setările de mai sus.
        visionPortal = builder.build();

        // Setează pragul de încredere pentru recunoașterile TFOD, în orice moment.
        tfod.setMinResultConfidence(0.75f);

        // Dezactivează sau re-activează procesorul TFOD în orice moment.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // sfârșit metoda initTfod()

    /**
     * Adaugă telemetrie despre recunoașterile obiectelor TensorFlow Object Detection (TFOD).
     */
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

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            double relativePosition = x - 320;

            String conePosition = determineConePosition(relativePosition);

            telemetry.addData("Poziția Conului", conePosition);

            // Planifică o traiectorie în funcție de poziția conului
            planTrajectory(conePosition);

            // Executează traiectoria
            drive.followTrajectoryAsync(trajectory);
        }
    }

    private String determineConePosition(double relativePosition) {
        // Definiți pozițiile celor trei benzi (stânga, mijloc, dreapta) în raport cu centrul vederii camerei.
        double LEFT_STRIP_POSITION = -100; // Ajustați în funcție de dimensiunile robotului dvs.
        double RIGHT_STRIP_POSITION = 100; // Ajustați în funcție de dimensiunile robotului dvs.

        if (relativePosition < LEFT_STRIP_POSITION) {
            return "Stânga";
        } else if (LEFT_STRIP_POSITION <= relativePosition && relativePosition <= RIGHT_STRIP_POSITION) {
            return "Mijloc";
        } else {
            return "Dreapta";
        }
    }

    private Trajectory trajectory;

    private void planTrajectory(String conePosition) {
        // Planificați traiectoria în funcție de poziția conului
        Pose2d startPose = drive.getPoseEstimate();
        TrajectoryBuilder builder = drive.trajectoryBuilder(startPose);

        if (conePosition.equals("Stânga")) {
            builder.lineToLinearHeading(new Pose2d(12, 24, Math.toRadians(90)));
        } else if (conePosition.equals("Dreapta")) {
            builder.lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(-90)));
        } else {
            builder.forward(12);
        }

        trajectory = builder.build();
    }
}