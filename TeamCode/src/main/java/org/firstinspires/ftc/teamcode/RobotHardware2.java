package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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


    public double pid;
    boolean manualControlBrat = false;
    boolean manualControlLift = false;
    PIDController pidController = new PIDController(0, 0, 0);

    public static double kpBrat = 0.005, kiBrat = 0, kdBrat = 0, ffBrat = 0.016;
    public static double kpLift = 0.003, kiLift = 0, kdLift = 0, ffLift = 0.01;

    public static int liftTarget = 0, BratTarget = 0;

    public static float GhearaSus = 1;
    public static float GhearaJos = 0.5F;

//    public Servo GhearaStanga , GhearaDreapta , GhearaInclinatie;
//
//    double closeClaw = 0.5, openClaw = 0;
//    double InclineClawRetract = -1;
//    double InclineClawParallel = 0.8;
//    boolean buttonIsPressed = false, toggleClaw = false, button2IsPressed = false, toggleInclineClaw = false, toggleDrone = false, button3IsPressed = false;

    public IMU imu;

    public RobotHardware2(HardwareMap hardwareMap){

        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront_OdometryLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack_OdometryFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront_OdometryRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        Brat = hardwareMap.get(DcMotorEx.class, "Brat");

        Brat.setDirection(DcMotorEx.Direction.FORWARD);

        Brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Brat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Brat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lift = hardwareMap.get(DcMotorEx.class, "Lift");

        Lift.setDirection(DcMotorEx.Direction.FORWARD);

        Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


//        GhearaStanga = hardwareMap.get(Servo.class, "GhearaStanga");
//        GhearaDreapta = hardwareMap.get(Servo.class, "GhearaDreapta");
//
//        GhearaInclinatie = hardwareMap.get(Servo.class, "GhearaInclinatie");
//
//        GhearaInclinatie.setDirection(Servo.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void LiftPID(Gamepad gamepad) {

        double ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLift)*0.25;
        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
            manualControlLift = true;
        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
            ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLift)*0.4;

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

        double ManualBratPower = 0.5;
        if(gamepad.right_stick_x != 0.0)
            manualControlBrat = true;

        pidController.setPID(kpBrat, kiBrat, kdBrat);
        int BratPos = Brat.getCurrentPosition();
        double pid = pidController.calculate(BratPos, BratTarget);
        double pidPowerBRAT = pid + ffBrat;

        if (manualControlBrat) {
            Brat.setPower(ManualBratPower);
        } else {
            Brat.setPower(pidPowerBRAT);
        }
    }

    public int ReturnPosLift(){
        return liftTarget;
    }
    public int ReturnPosBrat(){ return BratTarget; }

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