package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RobotHardware {
    public DcMotorEx
            leftFront, leftRear,
            rightRear, rightFront,
            Lift,
            BratDreapta, BratStanga;
    public Servo
            GhearaStanga, GhearaDreapta,
            GhearaInclinatie, Drona;

    double closeClaw = 0.5, openClaw = 0;
    double InclineClawRetract = -1;
    double InclineClawParallel = 0.8;
    boolean buttonIsPressed = false, toggleClaw = false, button2IsPressed = false, toggleInclineClaw = false, toggleDrone = false, button3IsPressed = false;
    public double pid;
    boolean manualControl = false;
    PIDController pidController = new PIDController(0, 0, 0);

    public static double kpLIFT = 0, kiLIFT = 0, kdLIFT = 0, ffLIFT = 0;
    public static int liftTarget = 0;
    public static double kpBRAT = 0.1, kiBRAT = 0, kdBRAT = 0, ffBRAT = 0.3;
    public static int BratTarget = 0;

    int high = 0, medium = 0, low = 0;
    double InitDrone = 0.5;
    int LaunchPos = 1;

    public RobotHardware(HardwareMap hardwareMap) {

        //region MotoareDrive
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

        //endregion

        //region MotoareLift
        Lift = hardwareMap.get(DcMotorEx.class, "Lift");

        BratStanga = hardwareMap.get(DcMotorEx.class, "BratStanga");
        BratDreapta = hardwareMap.get(DcMotorEx.class, "BratDreapta");

        Lift.setDirection(DcMotorEx.Direction.FORWARD);

        BratStanga.setDirection(DcMotor.Direction.FORWARD);
        BratDreapta.setDirection(DcMotor.Direction.FORWARD);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BratStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BratDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //endregion

        //region ServoGheara

        GhearaStanga = hardwareMap.get(Servo.class, "GhearaStanga");
        GhearaDreapta = hardwareMap.get(Servo.class, "GhearaDreapta");

        Drona = hardwareMap.get(Servo.class, "Drona");

        GhearaStanga.setDirection(Servo.Direction.FORWARD);
        GhearaDreapta.setDirection(Servo.Direction.REVERSE);

        Drona.setDirection(Servo.Direction.FORWARD);

        GhearaInclinatie = hardwareMap.get(Servo.class, "GhearaInclinatie");

        GhearaInclinatie.setDirection(Servo.Direction.FORWARD);



        //endregion

        //LynxBulkCachingMode
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    public void RetractClaw() {
        InclineClawState(InclineClawRetract);
    }

    public void ParallelClaw() {
        InclineClawState(InclineClawParallel);
    }


    public void ClawStateIndivi(double pos,boolean stg) {
        if(stg)
        GhearaStanga.setPosition(pos);
        else
        GhearaDreapta.setPosition(pos);
    }


    public void ClawState(double pos) {
        GhearaStanga.setPosition(pos);
        GhearaDreapta.setPosition(pos);
    }

    public void InitDrone() {
        Drona.setPosition(InitDrone);
    }

    public void LaunchDrone() {
        Drona.setPosition(LaunchPos);
    }

    public void InclineClawState(double pos) {
        GhearaInclinatie.setPosition(pos);
    }

    public void CloseClaw() {
        ClawState(closeClaw);
    }

    public void OpenClaw() {
        ClawState(openClaw);
    }

//    public void Drone(Gamepad gamepad) {
//        if(gamepad.y)
//            LaunchDrone();
//    }

    public void DriveMovement(Gamepad gamepad) {
        double Forward = -gamepad.left_stick_y;
        double Strafe = gamepad.left_stick_x;
        double Turn = gamepad.right_stick_x;

        if (!gamepad.left_bumper) {
            Strafe /= 2;
            Forward /= 2;
        }
        if (!gamepad.right_bumper) {
            Turn /= 2;
        }
        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + Turn;
        final double v2 = (r * Math.sin(robotAngle)) - Turn;
        final double v3 = (r * Math.sin(robotAngle)) + Turn;
        final double v4 = (r * Math.cos(robotAngle)) - Turn;
    //v1,v2,v3,v4

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }

    public void LiftPID(Gamepad gamepad) {

        double ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLIFT)*0.5;
        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
            manualControl = true;
        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
            ManualLiftPower = (gamepad.left_trigger-gamepad.right_trigger+ffLIFT)*0.7;

        pidController.setPID(kpLIFT, kiLIFT, kdLIFT);
        int LiftPos = Lift.getCurrentPosition();
        double pid = pidController.calculate(LiftPos, liftTarget);
        double pidPowerLIFT = pid + ffLIFT;

        if (manualControl) {
            Lift.setPower(ManualLiftPower);
//            LiftDreapta.setPower(manualPower);
//            LiftStanga.setPower(manualPower);
        } else {
            Lift.setPower(pidPowerLIFT);
//            LiftDreapta.setPower(pidPower);
//            LiftStanga.setPower(pidPower);
        }
    }


    public void BratPID(Gamepad gamepad) {

//        double manualArmPower = (gamepad.left_stick_y - gamepad.right_stick_y + ffBRAT) * 0.5;
//
//        if (gamepad.left_stick_y > 0.1 || gamepad.right_stick_y > 0.1)
//            manualControl = true;
//        if (gamepad.left_stick_y > 0.9 || gamepad.right_stick_y > 0.9)
//            manualArmPower = (gamepad.left_stick_y - gamepad.right_stick_y + ffBRAT) * 0.7;

//       double manualArmPower = (gamepad.left_trigger - gamepad.right_trigger + ffBRAT) * 0.2;
//
//        if (gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
//            manualControl = true;
//        if (gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
//            manualArmPower = (gamepad.left_trigger - gamepad.right_trigger + ffBRAT) * 0.5;

        pidController.setPID(kpBRAT, kiBRAT, kdBRAT);

        int armPos = BratStanga.getCurrentPosition();
        double pid = pidController.calculate(armPos, BratTarget);
        double pidPowerBrat = pid + ffBRAT;

        BratDreapta.setPower(pidPowerBrat);
        BratStanga.setPower(pidPowerBrat);

        if(gamepad.dpad_up)
            BratTarget = BratTarget + 1;
        if(gamepad.dpad_down)
            BratTarget = BratTarget - 1;

//        if (manualControl) {
//            BratDreapta.setPower(manualArmPower);
//            BratStanga.setPower(manualArmPower);
//        }
//        else {
//            BratDreapta.setPower(pidPowerBrat);
//            BratStanga.setPower(pidPowerBrat);
//        }

    }
    public void DroneManager(Gamepad gamepad) {
        if (gamepad.y && !button3IsPressed) {
            if (toggleDrone)
                Drona.setPosition(InitDrone);
            else
                Drona.setPosition(LaunchPos);
            button3IsPressed = true;
            toggleDrone = !toggleDrone;
        } else if (!gamepad.y)
            button3IsPressed = false;
    }

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

//    public void InitBratLift() {
//        ServoStanga.setPosition(0);
//        ServoDreapta.setPosition(1);
//    }
//
//    public void ArmPos(Gamepad gamepad)  {
//        if(gamepad.a) {
//            ServoStanga.setPosition(0);
//            ServoDreapta.setPosition(1);
//        }
//        if(gamepad.b) {
//            ServoStanga.setPosition(1);
//            ServoDreapta.setPosition(0);
//        }
//        if(gamepad.y) {
//            ServoStanga.setPosition(0.8);
//            ServoDreapta.setPosition(0.2);
//        }
//    }

    public void setLiftTarget(int pos) {
        liftTarget = pos;
    }

    public void setBratTarget(int pos) {
        BratTarget = pos;
    }

    public void ClawManager(Gamepad gamepad) {

        if (gamepad.x && !buttonIsPressed) {
            if (toggleClaw)
                ClawState(closeClaw);
            else
                ClawState(openClaw);
            buttonIsPressed = true;
            toggleClaw = !toggleClaw;
        } else if (!gamepad.x)
            buttonIsPressed = false;

    }

    public void ClawStanga(Gamepad gamepad) {

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

    public int getLiftLeftPosition() {
        return Lift.getCurrentPosition();
//        return LiftStanga.getCurrentPosition();
    }

    public int getArmPosition() {
        return BratStanga.getCurrentPosition();
    }

//    public int getLiftRightPosition()
//    {
//        return Lift.getCurrentPosition();
////        return LiftDreapta.getCurrentPosition();
//    }

    public int getArmTarget() {
        return BratTarget;
    }

        public int getLiftTarget ()
        {
            return liftTarget;
        }
        public double getArmPower() {
            return BratStanga.getPower();
        }

        public double getLiftPower ()
        {
            return Lift.getPower();
//        return LiftDreapta.getPower();
        }
    }

