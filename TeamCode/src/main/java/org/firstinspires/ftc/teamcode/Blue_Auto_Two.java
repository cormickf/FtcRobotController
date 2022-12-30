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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name=" Blue Auto Two", group="Linear Opmode")

public class Blue_Auto_Two extends LinearOpMode {


    private static final double WHEEL_CIRCUMFERENCE = 3.5433 * Math.PI;
    public static  final double TICKS_PER_INCH = 560 / WHEEL_CIRCUMFERENCE;
    private static final int Turn_Time_90 = 1265;

    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorBackLeft = null;

    private Servo claw;



    private void Waitmilli(int milli){
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()&& timer.milliseconds() < milli){


        }


    }

    private void DriveInches(double inches, double power) {
        final int ticksToDrive = (int) (inches * TICKS_PER_INCH);

        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticksToDrive);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(power);


        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticksToDrive);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(power);


        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticksToDrive);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(power);


        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticksToDrive);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(power);


        while(opModeIsActive() && motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy());

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void StrafeInchesRight(double inches, double power) {
        final int ticksToDrive = (int) (inches * TICKS_PER_INCH);

        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticksToDrive);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(power);


        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() - ticksToDrive);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(power);


        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() - ticksToDrive);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(power);


        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticksToDrive);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(power);


        while(opModeIsActive() && motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy());

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void StrafeInchesLeft(double inches, double power) {
        final int ticksToDrive = (int) (inches * TICKS_PER_INCH);

        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() - ticksToDrive);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(power);


        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticksToDrive);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(power);


        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticksToDrive);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(power);


        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() - ticksToDrive);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(power);


        while(opModeIsActive() && motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy());

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }





    @Override
    public void runOpMode() {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);



        claw = hardwareMap.servo.get("Claw");

        waitForStart();
        claw.setPosition(1);
        DriveInches(-108,.7);
        StrafeInchesLeft(30,.7);
        DriveInches(5, .5);
        claw.setPosition(.25);





        




    }
}
