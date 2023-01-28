package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Constant;

@TeleOp(name = "Main TeleOp 2", group = "TeleOp")
public class MainTeleOp2 extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight,Slider,Arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;
    double driveSpeed;



    // list of methods

    // arm to drop position
    public void armBack() {
        Arm.setTargetPosition(Constants.armBack);
    }

    // arm to pick up position
    public void armFront() {
        Arm.setTargetPosition(0);
    }

    // wrist to pick up position
    public void wristDown() {
        wrist.setPosition(Constants.wristDown);
    }

    // wrist to drop position
    public void wristUp() {
        wrist.setPosition(Constants.wristUp);
    }

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        FrontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        FrontRight = hardwareMap.get(DcMotorEx.class, "front right");
        BackLeft = hardwareMap.get(DcMotorEx.class, "back left");
        BackRight = hardwareMap.get(DcMotorEx.class, "back right");
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        Slider = hardwareMap.get(DcMotorEx.class, "slider");
        Arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Slider.setDirection(DcMotorSimple.Direction.REVERSE);

        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        resetRuntime();

        while(opModeIsActive()){

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // drive speeds
            if(gamepad1.left_trigger > 0.15) {
                driveSpeed = Constants.driveSpeedSlow;
            } else if (gamepad1.right_trigger > 0.15) {
                driveSpeed = Constants.driveSpeedFast;
            } else {
                driveSpeed = Constants.driveSpeedNormal;
            }

            // only drives when input is there
            if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1){
                Drive(vert,horz,rotate);
            }
            else{
                Drive(0,0,0);
            }

            // grabber closed preset
            if(gamepad1.left_bumper){
                grabber.setPosition(Constants.grabberClose);
            }

            // grabber open preset
            if(gamepad1.right_bumper){
                grabber.setPosition(Constants.grabberOpen);
            }

            if(gamepad1.y) {
                wristUp();
            }

            if(gamepad1.x) {
                wristDown();
            }

            if(gamepad2.dpad_up) {
                Slider.setTargetPosition(Slider.getCurrentPosition()+30);
            }

            if(gamepad2.dpad_down) {
                Slider.setTargetPosition(Slider.getCurrentPosition()-30);
            }

            // Everything down
            if(gamepad2.a) {
                Slider.setPower(0.9);
                Slider.setTargetPosition(0);
                Arm.setPower(0.55);
                armFront();
                wristDown();
            }

            // arm back + slider up to low
            if(gamepad2.b) {
                Slider.setPower(0.5);
                Slider.setTargetPosition(Constants.slideLow);
                Arm.setPower(0.75);
                armBack();
                wristUp();
            }

            if(gamepad2.y) {
                Slider.setPower(0.5);
                Slider.setTargetPosition(Constants.slideMedium);
                Arm.setPower(0.6);
                armBack();
                wristUp();
            }

            // telemetry for testing
            telemetry.addData("slider position", Slider.getCurrentPosition());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + Arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    // drive calculations

    double driveTuningFR = 1.0;
    double driveTuningFL = 0.78;
    double driveTuningBR = 1.0;
    double driveTuningBL = 0.8;

    public void Drive(double vert, double horz, double rotate){
        double frdrive = -vert - horz - rotate;
        double fldrive = -vert + horz + rotate;
        double brdrive = -vert + horz - rotate;
        double bldrive = -vert - horz + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        FrontRight.setPower(driveSpeed * driveTuningFR * frdrive / max);
        FrontLeft.setPower(driveSpeed * driveTuningFL * fldrive / max);
        BackRight.setPower(driveSpeed * driveTuningBR * brdrive / max);
        BackLeft.setPower(driveSpeed * driveTuningBL * bldrive / max);









    }
}
