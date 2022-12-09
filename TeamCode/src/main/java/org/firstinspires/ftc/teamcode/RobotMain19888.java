package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class RobotMain19888 extends LinearOpMode{


    private DcMotor lf = null, lr = null, rf = null, rr = null, mid = null; //Driving Motors
    private DcMotor slideLeft = null, slideRight = null;                    //Slide Motors
    protected Servo leftHand = null, rightHand = null;                      //Servos


    public void ROBOT_INITIALIZE(HardwareMap HwMap){
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");
        mid = hardwareMap.dcMotor.get("mid");

        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");

        leftHand = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");



        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*============================TeleOp Code================================*/


    public double powerfactor(){
        if (gamepad1.left_bumper){
            return 1;
        }
        else{
            return 0.7;
        }
    }

    public boolean TeleStraight() {
        double leftWheels, rightWheels;
        leftWheels = gamepad1.left_stick_y*(1+gamepad1.right_stick_x);
        rightWheels=gamepad1.left_stick_y*(1-gamepad1.right_stick_x);

        lf.setPower(-leftWheels);
        lr.setPower(-leftWheels);
        rf.setPower(-rightWheels);
        rr.setPower(-rightWheels);

            return true;
    }
    public void TeleStrafe() {
        if (gamepad1.left_stick_x>0.25 ||gamepad1.left_stick_x<-0.25) {
            mid.setPower(-gamepad1.left_stick_x);}
        else {
            mid.setPower(0);
        }
    }

    public void TeleTurn() {
        double power = 0;
        if (gamepad1.right_stick_x>0){
            power = (gamepad1.right_stick_x)*(gamepad1.right_stick_x) * powerfactor();
            lf.setPower(power);
            lr.setPower(power);
            rf.setPower(-power);
            rr.setPower(-power);
        }
        else if (gamepad1.right_stick_x<0){
            power = -(gamepad1.right_stick_x)*(gamepad1.right_stick_x) * powerfactor();
            lf.setPower(power);
            lr.setPower(power);
            rf.setPower(-power);
            rr.setPower(-power);
        }
        else {
            power =0;
            lf.setPower(power);
            lr.setPower(power);
            rf.setPower(-power);
            rr.setPower(-power);
        }

    }

    public void TeleSlide() {
        if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2 || gamepad1.a || gamepad2.a) {
            slideLeft.setPower(1);
            slideRight.setPower(1);
        }
        else if (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2 || gamepad1.y || gamepad2.y){
            slideLeft.setPower(-1);
            slideRight.setPower(-1);

        }
        else{
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
    }

    public void TeleClaw() {
        if (gamepad2.left_bumper || gamepad1.x || gamepad2.x){
            leftHand.setPosition(0.6);
            rightHand.setPosition(0.4);
        }
        if (gamepad2.right_bumper || gamepad1.b || gamepad2.b){
            leftHand.setPosition(0.4);
            rightHand.setPosition(0.6);
        }
    }






}