package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ArmDecelerateThread extends Thread {
    private DcMotorEx arm;

    public ArmDecelerateThread(DcMotorEx arm) {
        arm = arm;
        this.setName("ArmBackThread");
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {

// start timer t
// if arm position < 1000, wait

// if arm position > 1000 and time < 2000 set arm power to .35
// if time > 2000 exit thread

        ElapsedTime t = new ElapsedTime(0);

        // loop until 2 seconds
        while (t.milliseconds() < 2000) {
            // if arm not at position, wait
            if (arm.getCurrentPosition() < 1000) {
                try {
                    wait(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                // if arm at position, set new power
                arm.setPower(0.35);
            }
        }
    }
}
