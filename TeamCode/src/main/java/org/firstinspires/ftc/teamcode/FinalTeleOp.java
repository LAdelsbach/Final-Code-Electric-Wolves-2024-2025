package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Current Final TeleOP", group = "Iterative Opmode")
@Config
public class FinalTeleOp extends OpMode{
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
    static public double in_claw_min = 0.01;
    static public double in_claw_max = 0.2;
    static public double h_slide_min = 0.05;
    static public double h_slide_max = 0.3;

    //Outake min/max values

    static public double arm_min = 0.06;
    static public double arm_max = 0.7;
    static public double out_claw_min = 0.05;
    static public double out_claw_max = 0.2;
    static public double out_claw_rot_min = 0.1;
    static public double out_claw_rot_max = 0.5;
    static public int linear_arm_max = 100;
    static public double grab_time_max = 0.5;
    static public double height_time_max = 1.0;
    static public double scan_time_max = 1.0;
    static public double transfer_time_max = 1.0;
    public ElapsedTime grab_time = new ElapsedTime();
    public ElapsedTime height_time = new ElapsedTime();
    public ElapsedTime scan_time = new ElapsedTime();
    public ElapsedTime transfer_time = new ElapsedTime();
    public FinalTeleOp(){
        super();
    }
    public void init() {
        scan_time.reset();
        transfer_time.reset();
        grab_time.reset();
        height_time.reset();

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
    private void movement() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
//        if(gamepad1.right_stick_y > 0.05){
//            rx = gamepad1.right_stick_x;}
//        else{
//            rx = 0;
//        }
        //This if statement is just because sometimes the controller is slightly triggered
        //In that case, if not purposefull, you do not want to go at 95% speed for no reason
        //adjust for imperfect strafing
        double adj = 1.1;
        x *= adj;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double speed_factor = 1;
        if (gamepad1.right_trigger > 0.05) {
            speed_factor = 1 - (0.8 * gamepad1.right_trigger);
        }
        motor_fl.setPower(frontLeftPower * speed_factor);
        motor_bl.setPower(backLeftPower * speed_factor);
        motor_fr.setPower(frontRightPower * speed_factor);
        motor_br.setPower(backRightPower * speed_factor);
    }
    public void game_specific() {

        if(gamepad2.dpad_up && scan_time.time()>scan_time_max){
            scan();
            scan_time.reset();
        }
        else if(gamepad2.dpad_down ){//&& transfer_time.time() > transfer_time_max
            transfer();
        }
        else if(gamepad2.dpad_right){
        transfer_2();
//    }
        }
        else if(gamepad2.y){
            drop();
        }
        else if(gamepad2.a){// && grab_time.time()>grab_time_max
            grab();
        }
        else if(gamepad2.b){
             arm_l.setPosition(arm_max);
             arm_r.setPosition(arm_max);
        }
        else if(gamepad2.x){
            out_claw_r.setPosition(out_claw_max);
            out_claw_l.setPosition(out_claw_max);
            arm_l.setPosition(arm_min);
            arm_r.setPosition(arm_min);
        }
        if(Math.abs(gamepad2.left_stick_y) > 0.01){
            linear_arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear_arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear_arm_right.setPower(gamepad2.left_stick_y);
            linear_arm_left.setPower(gamepad2.left_stick_y);

        }
        else{
            linear_arm_left.setPower(0);
            linear_arm_right.setPower(0);
        }
    }


    private void scan(){
        telemetry.addData("Scanning", 0);
        telemetry.addLine("Scanning");
        h_slide_r.setPosition(h_slide_max);
        h_slide_l.setPosition(h_slide_max);
        in_rotate.setPosition(in_rotate_ready);
        in_claw.setPosition(in_claw_max);


    }
    private void grab(){
        telemetry.addData("Grabbing", h_slide_l.getPosition());
        telemetry.addLine("Grabbing");
            in_claw.setPosition(in_claw_max);
            in_rotate.setPosition(in_rotate_min);
            in_claw.setPosition(in_claw_min);


    }
    private void transfer_2(){

        out_claw_l.setPosition(out_claw_min);
        out_claw_r.setPosition(out_claw_min);
        in_claw.setPosition(in_claw_max);
        in_rotate.setPosition(in_rotate_ready);
        arm_l.setPosition(arm_max);
        arm_r.setPosition(arm_max);

    }
    private void transfer(){
        telemetry.addData("Transfering", 0);
        telemetry.addLine("Transfering");
            in_rotate.setPosition(in_rotate_max);
            in_claw.setPosition(in_claw_min);
            out_claw_rot.setPosition(out_claw_rot_min);
            out_claw_r.setPosition(out_claw_max);
            out_claw_l.setPosition(out_claw_max);
            arm_l.setPosition(arm_min);
            arm_r.setPosition(arm_min);

            h_slide_r.setPosition(h_slide_min);
            h_slide_l.setPosition(h_slide_min);


    }



//
    private void drop(){
        telemetry.addData("Dropping", 0);
        telemetry.addLine("Dropping");
        out_claw_l.setPosition(out_claw_max);
        out_claw_r.setPosition(out_claw_max);
        if((out_claw_r.getPosition()-out_claw_max)>0.05){
            arm_l.setPosition(arm_min);
            arm_r.setPosition(arm_min);
            out_claw_l.setPosition(out_claw_min);
            out_claw_r.setPosition(out_claw_min);
        }
    }
    public void report_telemetry() {
//        telemetry.addData("Front Left", motor_fl.getPower());
//        telemetry.addData("Front Right", motor_fr.getPower());
//        telemetry.addData("Back Left", motor_bl.getPower());
//        telemetry.addData("Back Right", motor_br.getPower());
        telemetry.update();
    }
}

