package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp 2", group = "TeleOp")
public class MainTeleOp2 extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight, slider, arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;
    double driveSpeed;



    // list of methods

    // arm to drop position
    public void armBack() {
        arm.setTargetPosition(Constants.armBack);
    }

    // arm to pick up position
    public void armFront() {
        arm.setTargetPosition(0);
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
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();
        resetRuntime();

        while(opModeIsActive()){

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // drive speeds
            if(gamepad1.left_trigger > 0.15) {
                driveSpeed = Constants.driveSpeedSlow + (1.0-gamepad1.left_trigger) * (Constants.driveSpeedNormal-Constants.driveSpeedSlow);
            } else  if(gamepad1.right_trigger > 0.1){
                driveSpeed = Constants.driveSpeedNormal + gamepad1.right_trigger * (Constants.driveSpeedFast - Constants.driveSpeedNormal);
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
                slider.setTargetPosition(slider.getCurrentPosition()+30);
            }
            if(gamepad2.dpad_down) {
                slider.setTargetPosition(slider.getCurrentPosition()-30);
            }

            // Everything down
            if(gamepad2.a) {
                slider.setPower(1.0);
                slider.setTargetPosition(0);
                arm.setPower(0.8);
                armFront();
                wristDown();
            }

            // arm back + slider up to low
            if(gamepad2.b) {
                slider.setPower(0.7);
                slider.setTargetPosition(Constants.slideLow);
                arm.setPower(0.6);
                arm.setTargetPosition(Constants.armBack);
                wrist.setPosition(Constants.wristUp);

            }

            if(gamepad2.y) {
                slider.setPower(0.7);
                slider.setTargetPosition(Constants.slideMedium);
                arm.setPower(0.6);
                arm.setTargetPosition(Constants.armBack);
                wrist.setPosition(Constants.wristUp);

            }


            // telemetry for testing
            telemetry.addData("slider position", slider.getCurrentPosition());
            telemetry.addData("Grabber position: ", grabber.getPosition());
            telemetry.addData("Wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    // drive calculations

    double driveTuningFR = 1.0;
    double driveTuningFL = 0.75;
    double driveTuningBR = 1.0;
    double driveTuningBL = 0.75;

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
