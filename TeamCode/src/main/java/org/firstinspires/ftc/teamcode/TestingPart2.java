package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;


@TeleOp(name = "Testing_pat_2", group = "Iterative Opmode")
@Config
public class TestingPart2 extends OpMode{
    DcMotor motor_fl;
    DcMotor motor_fr;
    DcMotor motor_bl;
    DcMotor motor_br;
    DcMotor linear_arm_left;
    DcMotor linear_arm_right;

    Servo in_rotate;
    Servo h_slide_l;
    Servo h_slide_r;
    Servo in_claw;
    Servo arm_l;
    Servo arm_r;
    Servo out_claw_l;
    Servo out_claw_r;

    Servo out_claw_rot;


    //Intake min/max values
    static public double in_rotate_min = 0.1;
    static public double in_rotate_ready = 0.23;
    static public double in_rotate_max = 0.9;
    static public double in_claw_min = 0.02;
    static public double in_claw_max = 0.2;
    static public double h_slide_min = 0.05;
    static public double h_slide_max = 0.3;

    //Outake min/max values

    static public double arm_min = 0.07;
    static public double arm_max = 0.7;
    static public double out_claw_min = 0.05;
    static public double out_claw_max = 0.2;
    static public double out_claw_rot_min = 0.1;
    static public double out_claw_rot_max = 0.5;




    public TestingPart2(){
        super();
    }
    public void init() {
        motor_fl = hardwareMap.dcMotor.get("FL");
        motor_fr = hardwareMap.dcMotor.get("FR");
        motor_bl = hardwareMap.dcMotor.get("BL");
        motor_br = hardwareMap.dcMotor.get("BR");

        linear_arm_left = hardwareMap.dcMotor.get("lal");
        linear_arm_right = hardwareMap.dcMotor.get("lar");

        linear_arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear_arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_fl.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_fr.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_bl.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_br.setDirection(DcMotorSimple.Direction.FORWARD);

        linear_arm_left.setDirection(DcMotorSimple.Direction.FORWARD);
        linear_arm_right.setDirection(DcMotorSimple.Direction.REVERSE);
        linear_arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        in_rotate = hardwareMap.get(Servo.class, "ir");
        h_slide_r = hardwareMap.get(Servo.class, "h_r");
        h_slide_l = hardwareMap.get(Servo.class, "h_l");
        in_claw = hardwareMap.get(Servo.class, "ic");

        in_rotate.setDirection(Servo.Direction.FORWARD);
        h_slide_r.setDirection(Servo.Direction.FORWARD);
        h_slide_l.setDirection(Servo.Direction.REVERSE);
        in_claw.setDirection(Servo.Direction.REVERSE);

        arm_l = hardwareMap.get(Servo.class, "a_l");
        arm_r = hardwareMap.get(Servo.class, "a_r");

        out_claw_l = hardwareMap.get(Servo.class, "oc_l");
        out_claw_r = hardwareMap.get(Servo.class, "oc_r");
        out_claw_rot = hardwareMap.get(Servo.class, "ocr");

        arm_l.setDirection(Servo.Direction.REVERSE);
        arm_r.setDirection(Servo.Direction.FORWARD);

        out_claw_l.setDirection(Servo.Direction.REVERSE);
        out_claw_r.setDirection(Servo.Direction.FORWARD);
        out_claw_rot.setDirection(Servo.Direction.FORWARD);
    }
    public void loop() {
        control_robot();
        report_telemetry();
    }
    private void control_robot() {
        movement();
        game_specific();
    }
    private void movement(){
        if(gamepad1.dpad_up){
            arm_l.setPosition(arm_max);
            arm_r.setPosition(arm_max);

        }
        if(gamepad1.dpad_down) {
            arm_l.setPosition(arm_min);
            arm_r.setPosition(arm_min);
        }
        //close/open claw:
        if(gamepad1.dpad_left){
            out_claw_l.setPosition(out_claw_min);
            out_claw_r.setPosition(out_claw_min);
        }
        if(gamepad1.dpad_right){
            out_claw_l.setPosition(out_claw_max);
            out_claw_r.setPosition(out_claw_max);
        }
        if(gamepad1.a){
            out_claw_rot.setPosition(out_claw_rot_min);
        }
        if(gamepad1.y){
            out_claw_rot.setPosition(out_claw_rot_max);
        }
    }
    public void game_specific() {
        if(gamepad2.dpad_up){
            in_rotate.setPosition(in_rotate_min);
        }
        else if(gamepad2.dpad_down){
            in_rotate.setPosition(in_rotate_max);
        }
        else if(gamepad2.b){
            in_rotate.setPosition(in_rotate_ready);
        }
        if(gamepad2.dpad_left){
            h_slide_l.setPosition(h_slide_max);
            h_slide_r.setPosition(h_slide_max);

        }
        else if(gamepad2.dpad_right){
            h_slide_l.setPosition(h_slide_min);
            h_slide_r.setPosition(h_slide_min);
        }

        if(gamepad2.a) {
            in_claw.setPosition(in_claw_min);
        }
        else if(gamepad2.y){
            in_claw.setPosition(in_claw_max);
        }

    }
    public void report_telemetry() {
        telemetry.addData("Front Left", motor_fl.getPower());
        telemetry.addData("Front Right", motor_fr.getPower());
        telemetry.addData("Back Left", motor_bl.getPower());
        telemetry.addData("Back Right", motor_br.getPower());
        telemetry.update();
    }
}