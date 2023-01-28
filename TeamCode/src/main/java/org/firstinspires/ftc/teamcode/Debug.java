package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

            // drive calculations
            double vert = -gamepad1.left_stick_y;
            double horz = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;


            // only drives when input is there
            if(Math.abs(vert) > .1 || Math.abs(horz) > .1 || Math.abs(rotate) > .1){
                Drive(vert,horz,rotate);
            }
            else{
                Drive(0,0,0);
            }

            // arm adjustments

            if(gamepad1.a){
                Arm.setTargetPosition(Arm.getCurrentPosition() + 50);
            }
            if(gamepad1.b)
            {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 50);
            }

            // grabber adjustments

            if(gamepad1.left_bumper){
                grabber.setPosition(Constants.grabberOpen);
            }

            if(gamepad1.right_bumper){
                grabber.setPosition(Constants.grabberClose);
            }

            // slider adjustments

            if(gamepad1.dpad_up && Slider.getCurrentPosition() < 1500) {
                Slider.setTargetPosition(Slider.getCurrentPosition() + 100);
            }

            if(gamepad1.dpad_down && Slider.getCurrentPosition() > 100) {
                Slider.setTargetPosition(Slider.getCurrentPosition() - 100);
            }

            if(gamepad1.x) {
                wrist.setPosition(Constants.wristDown);
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

    // drive calculations

    double driveSpeed = 0.3;
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