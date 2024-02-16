package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class RobotHardware2 {
    ElapsedTime elapsedTime = new ElapsedTime();

    public DcMotorEx
                Brat, Lift , leftFront , leftRear , rightRear , rightFront;

    public Servo
                Inclin;

    public static int BratTarget = 0;

    public double pid;
    boolean manualControlBrat = false;
    boolean manualControlLift = false;
    PIDController pidController = new PIDController(0, 0, 0);

    public static double kpBrat = 0, kiBrat = 0, kdBrat = 0, ffBrat = 0;
    public static double kpLift = 0, kiLift = 0, kdLift = 0, ffLift = 0;

    public static int liftTarget = 0;

    public static float GhearaSus = 1;
    public static float GhearaJos = 0.5F;

    public Servo GhearaStanga , GhearaDreapta , GhearaInclinatie;

    double closeClaw = 0.5, openClaw = 0;
    double InclineClawRetract = -1;
    double InclineClawParallel = 0.8;
    boolean buttonIsPressed = false, toggleClaw = false, button2IsPressed = false, toggleInclineClaw = false, toggleDrone = false, button3IsPressed = false;

    public IMU imu;

    public RobotHardware2(HardwareMap hardwareMap){
        Brat = hardwareMap.get(DcMotorEx.class, "Brat");

        Brat.setDirection(DcMotorEx.Direction.FORWARD);

        Brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift = hardwareMap.get(DcMotorEx.class, "Lift");

        Lift.setDirection(DcMotorEx.Direction.FORWARD);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftPID(Gamepad gamepad) {

        double ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLift)*0.5;
        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
            manualControlLift = true;
        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
            ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLift)*0.7;

        pidController.setPID(kpLift, kiLift, kdLift);
        int LiftPos = Lift.getCurrentPosition();
        double pid = pidController.calculate(LiftPos, liftTarget);
        double pidPowerLIFT = pid + ffLift;

        if (manualControlLift) {
            Lift.setPower(ManualLiftPower);
        } else {
            Lift.setPower(pidPowerLIFT);
        }
    }

    public void BratPID(Gamepad gamepad) {

        double ManualBratPower = (gamepad.right_stick_x+ffBrat)*0.3;
        if(gamepad.right_stick_x != 0.0)
            manualControlBrat = true;

        pidController.setPID(kpBrat, kiBrat, kdBrat);
        int LiftPos = Brat.getCurrentPosition();
        double pid = pidController.calculate(LiftPos, liftTarget);
        double pidPowerLIFT = pid + ffBrat;

        if (manualControlBrat) {
            Brat.setPower(ManualBratPower);
        } else {
            Brat.setPower(pidPowerLIFT);
        }
    }

    public void ClawStateIndivi(double pos,boolean stg) {
        if(stg)
            GhearaStanga.setPosition(pos);
        else
            GhearaDreapta.setPosition(pos);}

    public void InclineClawManager(Gamepad gamepad) {

        if (gamepad.right_bumper && !button2IsPressed) {
            if (toggleInclineClaw)
                InclineClawState(InclineClawRetract);
            else
                InclineClawState(InclineClawParallel);
            button2IsPressed = true;
            toggleInclineClaw = !toggleInclineClaw;
        } else if (!gamepad.right_bumper)
            button2IsPressed = false;

    }

    public void ClawDreapta(Gamepad gamepad) {

        if (gamepad.b && !buttonIsPressed) {
            if (toggleClaw)
                ClawStateIndivi(closeClaw,false);
            else
                ClawStateIndivi(openClaw,false);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.b)
            buttonIsPressed = false;

    }

    public void ClawStanga  (Gamepad gamepad) {

        if (gamepad.x && !buttonIsPressed) {
            if (toggleClaw)
                ClawStateIndivi(closeClaw, true);
            else
                ClawStateIndivi(openClaw, true);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.x)
            buttonIsPressed = false;

    }

    public void InclineClawState(double pos) {
        GhearaInclinatie.setPosition(pos);
    }

    public void ClawLeftOpen() {
        IndividualClawLeftState(closeClaw);
    }

    public void ClawLeftClose() {
        IndividualClawLeftState(openClaw);
    }

    public void IndividualClawLeftState(double pos) {
        GhearaStanga.setPosition(pos);
    }

    public void IndividualClawRightState(double pos) {
        GhearaDreapta.setPosition(pos);
    }

    public void ClawRightOpen() {
        IndividualClawRightState(closeClaw);
    }

    public void ClawRightClose() {
        IndividualClawRightState(openClaw);
    }

    public void DriveMovement(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        if (!gamepad.left_bumper) {
            x /= 2;
            y /= 2;
        }
        if (!gamepad.right_bumper) {
            rx /= 2;
        }

        if (gamepad.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower = (rotY + rotX + rx);
        double backLeftPower = (rotY - rotX + rx);
        double frontRightPower = (rotY - rotX - rx);
        double backRightPower = (rotY + rotX - rx);

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}

