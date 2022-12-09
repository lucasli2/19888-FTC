package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class roboHardware {
    public DcMotor lf = null,lr = null,rf = null,rr = null,mid = null;     //Gets Drive Motors
    public DcMotor slideLeft = null, slideRight = null;
    public Servo leftHand = null,rightHand = null;

    public static final double LOW_SPEED        = 0.4;
    public static final double MID_SPEED        = 0.7;
    public static final double TOP_SPEED        = 0.7;
    public static final double ARM_UP_POWER     = 1.0;
    public static final double ARM_DOWN_POWER   = 1.0;
}

