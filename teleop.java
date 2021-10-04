/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="teleopController", group="Linear Opmode")
//@Disabled
public class teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private arm arm = new arm ();
    hardwareRobot robot = new hardwareRobot();


    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor armMotor = null;
    private Servo clawServo = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        double speedVariance = 1;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double leftfrontPower;   // Power for forward and back motion
            double leftbackPower;  // Power for left and right motion
            double rightfrontPower;  // Power for rotating the robot
            double rightbackPower;

            //STRAFING

            double strafe = speedVariance*gamepad1.left_stick_x;
            double drive = speedVariance*gamepad1.left_stick_y;
            double rotate = speedVariance*gamepad1.right_stick_x;

            leftfrontPower = Range.clip(drive + strafe + rotate, -1.0, 1.0);
            rightfrontPower  = Range.clip(drive - strafe - rotate, -1.0, 1.0);
            leftbackPower  = Range.clip(drive + strafe - rotate, -1.0, 1.0);
            rightbackPower   = Range.clip(drive - strafe + rotate, -1.0, 1.0);



            //GAME PAD 1
            leftFrontMotor.setPower(leftfrontPower);
            leftBackMotor.setPower(leftbackPower);
            rightFrontMotor.setPower(rightfrontPower);
            rightBackMotor.setPower(rightbackPower);



            if (gamepad1.x) {
                arm.clawOpen();
            }
            if (gamepad1.y) {
                arm.clawClosed();
            }



            if (gamepad1.a){
                speedVariance =0.5;
            }

            if (gamepad1.b){
                speedVariance =1.0;
            }


            if (gamepad1.dpad_up) {
                armMotor.setPower(1.0);
            }
            if (gamepad1.dpad_down) {
                armMotor.setPower(-1.0);
            }
            else {
                armMotor.setPower(0.0);
            }





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftfrontPower, leftbackPower, rightfrontPower, rightbackPower);
            telemetry.update();
        }
    }
}
