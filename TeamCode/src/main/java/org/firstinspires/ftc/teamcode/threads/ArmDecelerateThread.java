package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmDecelerateThread extends Thread {
    private DcMotorEx arm;

    public ArmDecelerateThread(DcMotorEx arm) {
        arm = arm;
    }

    @Override
    public void run() {
        ElapsedTime t = new ElapsedTime(0);

        // loop until 2 seconds
        while (t.milliseconds() < 2000) {
            // if arm not at position, wait
            if (arm.getCurrentPosition() < 1000) {
                try {
                    wait(10);
                } catch (InterruptedException e) {
                    break;
                }
            } else {
                // if arm at position, set new power
                arm.setPower(0.35);
                break;
            }
        }
    }
}
