package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Debug", group = "TeleOp")
public class Debug extends LinearOpMode {
    DcMotorEx FrontLeft,FrontRight,BackLeft,BackRight,Slider,Arm;
    Servo grabber;
    Servo wrist;
    //DistanceSensor distance;

    // list of power values
    public double slidePower = 0.2;

    public double armPower = 0.2;

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

        // slider declarations

        Slider.setDirection(DcMotorSimple.Direction.REVERSE);

        Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setTargetPosition(0);
        Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider.setPower(slidePower);

        // arm declarations

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(armPower);



        waitForStart();

        while(opModeIsActive()){

            // arm adjustments

            if(gamepad1.a){
                Arm.setTargetPosition(Arm.getCurrentPosition() + 50);
            }
            if(gamepad1.b)
            {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 50);
            }

            // grabber adjustments

            if(Math.abs(gamepad1.left_stick_x) > 0.1) {
                grabber.setPosition((gamepad1.left_stick_x + 1.0)/2.0);
            }

            // slider adjustments

            if(gamepad1.dpad_up && Slider.getCurrentPosition() < 1500) {
                Slider.setTargetPosition(Slider.getCurrentPosition() + 100);
            }

            if(gamepad1.dpad_down && Slider.getCurrentPosition() > 100) {
                Slider.setTargetPosition(Slider.getCurrentPosition() - 100);
            }

            if(Math.abs(gamepad1.right_stick_x) > 0.1) {
                wrist.setPosition((gamepad1.right_stick_x + 1.0)/2.0);
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
            telemetry.addData("Slider position", Slider.getCurrentPosition());
            telemetry.addData("grabber position: ", grabber.getPosition());
            telemetry.addData("wrist position: ", wrist.getPosition());
            telemetry.addData("Arm position: ", + Arm.getCurrentPosition());
            //telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}