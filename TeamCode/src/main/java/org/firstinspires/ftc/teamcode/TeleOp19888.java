package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
**This is the code for TeleOp period**
make sure to change configuration on the phone before switching between TestTele and TeleOp
 */

@TeleOp
public class TeleOp19888 extends RobotMain19888 {


    public void runOpMode() throws InterruptedException {

        waitForStart();

        ROBOT_INITIALIZE(hardwareMap);

        while (opModeIsActive()) {
            if (!TeleStraight()) {
                TeleTurn();
            }
            //TeleStrafe();
            TeleSlide();
            TeleClaw();
            TeleStrafe();

            telemetry.addData("L Joystick x", gamepad1.left_stick_x);
            telemetry.addData("L Joystick y", gamepad1.left_stick_y);
            telemetry.addData("R Joystick x", gamepad1.right_stick_x);
            telemetry.addData("R Joystick y", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}