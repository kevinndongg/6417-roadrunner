package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Debug", group = "TeleOp")
public class Debug extends LinearOpMode {
    DcMotorEx frontLeft, frontRight, backLeft, backRight, slider, arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;

    // list of power values
    public double slidePower = 0.2;

    public double armPower = 0.2;

    double driveSpeed = Constants.driveSpeedSlow;

    public void runOpMode() throws InterruptedException
    {

        // motor declarations
        frontLeft = hardwareMap.get(DcMotorEx.class,"front left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back left");
        backRight = hardwareMap.get(DcMotorEx.class, "back right");
        //distance =  hardwareMap.get(DistanceSensor.class, "distance");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        // servo declarations
        wrist = hardwareMap.get(Servo.class, "wrist");
        grabber = hardwareMap.get(Servo.class,"grabber");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // slider declarations

        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(slidePower);

        // arm declarations

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);



        waitForStart();

        while(opModeIsActive()){
            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            // arm adjustments

            if(gamepad1.a){
                arm.setTargetPosition(arm.getCurrentPosition() + 50);
            }
            if(gamepad1.b)
            {
                arm.setTargetPosition(arm.getCurrentPosition() - 50);
            }

            // grabber adjustments

            if(gamepad1.left_trigger > 0.1) {
                if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                    grabber.setPosition((-gamepad1.left_stick_y + 1.0) / 2.0);
                }

                if (Math.abs(gamepad1.right_stick_y) > 0.1) {
                    wrist.setPosition((-gamepad1.right_stick_y + 1.0) / 2.0);
                }
            } else {
                Drive(vert, horz, rotate);
            }

            // slider adjustments

            if(gamepad1.dpad_up) {
                slider.setTargetPosition(slider.getCurrentPosition() + 100);
            }

            if(gamepad1.dpad_down && slider.getCurrentPosition() > 100) {
                slider.setTargetPosition(slider.getCurrentPosition() - 100);
            }


            if(gamepad1.y) {
                wrist.setPosition(Constants.wristUp);
            }

            // RESET ENCODERS

            /*
            if(gamepad1.y) {
                Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Slider.setTargetPosition(0);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

             */

            // telemetry for testing
            telemetry.addData("Slider position", slider.getCurrentPosition());
            telemetry.addData("grabber position: ", grabber.getPosition());
            telemetry.addData("wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    public void Drive(double vert, double horz, double rotate){
        double frdrive = -vert - horz - rotate;
        double fldrive = -vert + horz + rotate;
        double brdrive = -vert + horz - rotate;
        double bldrive = -vert - horz + rotate;

        // finding maximum drive for division below
        double max = Math.abs(Math.max(Math.abs(frdrive),Math.max(Math.abs(fldrive),Math.max(Math.abs(brdrive),Math.abs(bldrive)))));

        // power calculations
        frontRight.setPower(driveSpeed * Constants.driveTuningFR * frdrive / max);
        frontLeft.setPower(driveSpeed * Constants.driveTuningFL * fldrive / max);
        backRight.setPower(driveSpeed * Constants.driveTuningBR * brdrive / max);
        backLeft.setPower(driveSpeed * Constants.driveTuningBL * bldrive / max);
    }
}