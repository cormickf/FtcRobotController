/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Background.FtcGamePad;


@TeleOp(name = "Self Balance V2")

public class SelfBalanceV2 extends LinearOpMode {

    IMU imu;
    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;
    private int changer;


    private int multiplyer = 1000;
    private double perfectHeading = 0;


    private double initKp = 2;
    private double initKi = 12;
    private double initKd = 2;

    private double Kp = initKp;
    private double Ki = initKi;
    private double Kd = initKd;


    private double integralSum = 0;
    private double lastError = 0;


    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(IMU.class, "imu");


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        imu.initialize(new IMU.Parameters(orientationOnRobot));


        driverGamePad = new FtcGamePad("Driver GamePad", gamepad1, this::OnDriverGamePadChange);

        // Declare our motors
        // Make sure your ID's match your configuration


        DcMotorEx motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        DcMotorEx motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");



        waitForStart();

        // Loop and update the dashboard
        while (!isStopRequested()) {


            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();


            double leftMotorLocation = motorLeft.getCurrentPosition();
            double rightMotorLocation = motorRight.getCurrentPosition();


            // Read inverse IMU heading, as the UMG heading is CW positive
            double botHeading = orientation.getPitch(AngleUnit.DEGREES);

            double error = (perfectHeading - botHeading);
//
            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            integralSum = Range.clip(integralSum, -5, 5);
            out /= 45;

            out = Range.clip(out, -1, 1);
            motorLeft.setPower(out);
            motorRight.setPower(out);

            lastError = error;
            timer.reset();

            driverGamePad.update();


            telemetry.addData("Bot Heading", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Bot error", error);
            telemetry.addData("Out", out);
            telemetry.addData("Mult", multiplyer);
            telemetry.addData(changer == 1 ? " > Kp": "Kp",  Double.toString(Kp));
            telemetry.addData(changer == 2 ? " > Ki": "Ki", Double.toString(Ki));
            telemetry.addData(changer == 3 ? " > Kd": "Kd", Double.toString(Kd));
            telemetry.update();

            telemetry.update();
        }
    }
        private void OnDriverGamePadChange (FtcGamePad ftcGamePad,int button, boolean pressed){

            switch (button) {
                case FtcGamePad.GAMEPAD_A:
                    if (pressed)
                        if (changer == 1) {
                            Kp -= (multiplyer / 1000.0);
                        } else if (changer == 2) {
                            Ki -= (multiplyer / 1000.0);
                        } else if (changer == 3) {
                            Kd -= (multiplyer / 1000.0);
                        }
                    break;


                case FtcGamePad.GAMEPAD_Y:
                    if (pressed) {
                        if (changer == 1) {
                            Kp = Kp + (multiplyer / 1000.0);
                        } else if (changer == 2) {
                            Ki = Ki + (multiplyer / 1000.0);
                        } else if (changer == 3) {
                            Kd = Kd + (multiplyer / 1000.0);
                        }
                    }
                    break;


                case FtcGamePad.GAMEPAD_DPAD_UP:
                    if (pressed) {
                        if (changer < 3) {
                            changer = changer + 1;
                            break;
                        }
                        break;
                    }


                case FtcGamePad.GAMEPAD_DPAD_DOWN:
                    if (pressed) {
                        if (changer > 0) {
                            changer = changer - 1;
                            break;
                        }

                        break;
                    }

                    // The button to reset and set the K values
                case FtcGamePad.GAMEPAD_RBUMPER:
                    if (pressed) {
                        Kp = initKp;
                        Ki = initKi;
                        Kd = initKd;
                    }
                    break;

                case FtcGamePad.GAMEPAD_BACK:
                    if (pressed) {
                        Kp = 0;
                        Ki = 0;
                        Kd = 0;
                    }
                    break;

                    // The multiplyer for changing the K values
                case FtcGamePad.GAMEPAD_B:
                    if (pressed) {
                        multiplyer *= 10;
                        if (multiplyer > 1000) {
                            multiplyer = 1000;
                        }
                        break;
                    }


                case FtcGamePad.GAMEPAD_X:
                    if (pressed) {
                        multiplyer /= 10;
                        if (multiplyer < 1) {
                            multiplyer = 1;
                        }
                        break;
                    }


            }
        }
    }


