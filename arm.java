package org.firstinspires.ftc.teamcode.Classes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class arm {

    private LinearOpMode currentOpMode;
    private Servo clawServo;
    private DcMotor armMotor;



    public void init(hardwareRobot robot, LinearOpMode opMode){

        currentOpMode = opMode;
        armMotor = robot.armMotor;
        clawServo.setPosition(1.0);
    }


    public void clawOpen(){
        clawServo.setPosition(1.0);
    }
    public void clawClosed(){
        clawServo.setPosition(0.0);
    }













}