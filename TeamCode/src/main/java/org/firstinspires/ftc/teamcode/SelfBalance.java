package org.firstinspires.ftc.teamcode;






import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Background.FtcGamePad;

@TeleOp(name="Self Balance")
public class SelfBalance extends LinearOpMode {

    private FtcGamePad driverGamePad;

    public BNO055IMU imu;
    private int changer;
    private double changeValue;


    // heading 1.604444503784


    private double offsetHeading = 0;

    private final double perfectHeading = 90;
    private boolean balanced;
    private int multiplyer = 1000;



    private double initKp = 1.9;
    private double initKi = 8.3;
    private double initKd =  .1125;

    private double Kp = initKp;
    private double Ki = initKi;
    private double Kd =  initKd;





    private double integralSum = 0;
    private double lastError = 0;



    ElapsedTime timer = new ElapsedTime();









    @Override
    public void runOpMode() throws InterruptedException {

        driverGamePad = new FtcGamePad("Driver GamePad", gamepad1, this::OnDriverGamePadChange);

        // Declare our motors
        // Make sure your ID's match your configuration



        DcMotorEx motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        DcMotorEx motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");


        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);









        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        waitForStart();

        while (opModeIsActive()) {

            double leftMotorLocation = motorLeft.getCurrentPosition();
            double rightMotorLocation = motorRight.getCurrentPosition();


            // Read inverse IMU heading, as the UMG heading is CW positive
            double botHeading = -imu.getAngularOrientation().thirdAngle;

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





            telemetry.addData("Bot Heading", botHeading);
            telemetry.addData("Bot error", error);
            telemetry.addData("Out", out);
            telemetry.addData("Changer ", changer);
            telemetry.addData("Mult", multiplyer);
            telemetry.addData("Kp", Double.toString(Kp));
            telemetry.addData("Ki", Double.toString(Ki));
            telemetry.addData("Kd", Double.toString(Kd));
            telemetry.update();

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
//            if(botHeading == perfectHeading){
//                balanced = true;
//            } else if(botHeading != perfectHeading){
//                balanced = false;
//               if(botHeading > perfectHeading && botHeading != perfectHeading){
////                    motorLeft.setPower(.20);
////                    motorRight.setPower(.20);
//                   double  x = ((perfectHeading / botHeading) * 100);
//                   if(x >= 1.90){
//                       motorLeft.setPower(.40);
//                       motorRight.setPower(.40);
//                   }
//              }


//                if(botHeading < perfectHeading && botHeading != perfectHeading){
////                    motorLeft.setPower(-.20);
////                    motorRight.setPower(-.20);
//                }

            }



//            telemetry.addLine(String.valueOf(botHeading));
//            telemetry.addLine(String.valueOf(balanced));
            telemetry.update();



        }

    private void OnDriverGamePadChange(FtcGamePad ftcGamePad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:
                if(pressed)
                    if(changer == 1) {
                        Kp -= (multiplyer / 1000);
                    } else if(changer == 2){
                        Ki -= (multiplyer / 1000);
                    } else if(changer == 3){
                        Kd -= (multiplyer / 1000);
                    }
                break;
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    if(changer == 1) {
                        Kp = Kp + (multiplyer / 1000);
                    } else if(changer == 2){
                        Ki = Ki + (multiplyer / 1000);
                    } else if(changer == 3){
                        Kd = Kd + (multiplyer / 1000);
                    }
                }
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    if (changer < 3) {
                        changer = changer + 1;
                        break;
                    }
                    break;
                }
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    if(changer > 0){
                        changer = changer - 1;
                        break;
                    }

                    break;
                }
            case FtcGamePad.GAMEPAD_RBUMPER:
                if(pressed){
                   Kp = initKp;
                   Ki = initKi;
                   Kd = initKd;
                }
                break;

            case FtcGamePad.GAMEPAD_BACK:
                if(pressed){
                    Kp = 0;
                    Ki= 0;
                    Kd = 0;
                }
                break;
            case FtcGamePad.GAMEPAD_B:
                if(pressed){
                    multiplyer *= 10;
                    if(multiplyer > 1000){
                        multiplyer = 1000;
                    }
                    break;
                }
            case FtcGamePad.GAMEPAD_X:
                if(pressed){
                    multiplyer /= 10;
                    if(multiplyer < 1){
                        multiplyer = 1;
                    }
                    break;
                }









        }
    }
    }




