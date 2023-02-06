package org.firstinspires.ftc.teamcode;






import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Background.FtcGamePad;

@TeleOp(name="Self Balance")
public class SelfBalance extends LinearOpMode {

    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;
    public BNO055IMU imu;


    // heading 1.583333730697632


    private double offsetHeading = 0;

    private final double perfectHeading = 1.583333;
    private boolean balanced;

    private double Kp;
    private double Ki;
    private double Kd;


    private double refrence = 0;
    private double integralSum = 0;
    private double lastError = 0;


    ElapsedTime timer = new ElapsedTime();



    private double correctVelocity = 130;






    @Override
    public void runOpMode() throws InterruptedException {



        // Declare our motors
        // Make sure your ID's match your configuration



        DcMotorEx motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        DcMotorEx motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");









        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
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

            double error = (botHeading - perfectHeading) * 1.7;
//            motorLeft.setPower(error);
//            motorRight.setPower(error);

            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            motorLeft.setPower(out);
            motorRight.setPower(out);





            telemetry.addData("Bot Heading", botHeading);
            telemetry.addData("Bot error", error);
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
    }




