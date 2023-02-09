package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp 2", group = "TeleOp")
public class MainTeleOp2 extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight, slider, arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;
    double driveSpeed;

    boolean armGoingUp, wristDownSlider;

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        Drive6417 drive = new Drive6417(hardwareMap);
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
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
                drive.setPowers(vert,horz,rotate,driveSpeed);
            }
            else{
                drive.setPowers(0,0,0,0);
            }



            // while arm going up and is back
            if(armGoingUp && arm.getCurrentPosition() > Constants.armTop) {
                // arm slow
                arm.setPower(Constants.armSlowSpeed);
                // reset armGoingUp
                armGoingUp = false;
            }

            if(wristDownSlider && slider.getCurrentPosition() > 300) {
                wristDownSlider = false;
                slider.setPower(0.5);
                slider.setTargetPosition(0);
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
                wrist.setPosition(Constants.wristUp);
            }
            if(gamepad1.x) {
                if(wrist.getPosition() > Constants.wristUp - 0.1) {
                    wristDownSlider = true;
                    slider.setPower(0.3);
                    slider.setTargetPosition(350);
                }
                wrist.setPosition(Constants.wristDown);
            }

            if(gamepad2.dpad_up) {
                wristDownSlider = false;
                slider.setPower(0.6);
                slider.setTargetPosition(slider.getCurrentPosition()+50);
            }
            if(gamepad2.dpad_down) {
                wristDownSlider = false;
                slider.setPower(0.8);
                slider.setTargetPosition(slider.getCurrentPosition()-50);
            }

            // Everything down
            if(gamepad2.a) {
                armGoingUp = false;
                wristDownSlider = false;
                slider.setPower(1.0);
                slider.setTargetPosition(0);
                arm.setPower(0.85);
                arm.setTargetPosition(0);
                wrist.setPosition(Constants.wristDown);
            }

            // arm back + slider up to low
            if(gamepad2.b) {
                armGoingUp = true;
                wristDownSlider = false;
                slider.setPower(0.7);
                slider.setTargetPosition(Constants.slideLow);
                arm.setPower(0.65);
                arm.setTargetPosition(Constants.armBack);
                wrist.setPosition(Constants.wristUp);

            }

            if(gamepad2.y) {
                armGoingUp = true;
                wristDownSlider = false;
                slider.setPower(0.75);
                slider.setTargetPosition(Constants.slideMedium);
                arm.setPower(0.65);
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

    public void Drive(double vert, double horz, double rotate){
        double frdrive = (-vert - horz - rotate) * Constants.driveTuningFR;
        double fldrive = (-vert + horz + rotate) * Constants.driveTuningFL;
        double brdrive = (-vert + horz - rotate) * Constants.driveTuningBR;
        double bldrive = (-vert - horz + rotate) * Constants.driveTuningBL;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        frontRight.setPower(driveSpeed  * frdrive / max);
        frontLeft.setPower(driveSpeed * fldrive / max);
        backRight.setPower(driveSpeed * brdrive / max);
        backLeft.setPower(driveSpeed * bldrive / max);
    }
}
