package org.firstinspires.ftc.teamcode;






import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Teleop")
public class MecanumTeleOp extends LinearOpMode {

    private FtcGamePad driverGamePad;
    private FtcGamePad operatorGamePad;
    public BNO055IMU imu;
    public boolean fastMode;
    private boolean slowMode;





    private double offsetHeading = 0;


    private DcMotorEx motorLift;
    private Servo claw;




    @Override
    public void runOpMode() throws InterruptedException {

        driverGamePad = new FtcGamePad("Driver GamePad", gamepad1, this::OnDriverGamePadChange);
        operatorGamePad = new FtcGamePad("Operator GamePad", gamepad2, this::OnOperatorGamePadChange);

        // Declare our motors
        // Make sure your ID's match your configuration


        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorLift = hardwareMap.get(DcMotorEx.class, "motorLift");

        motorLift.setTargetPositionTolerance(20);

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("Claw");

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                     = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit                = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled           = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = Range.clip(Math.pow(gamepad1.left_stick_y, 3), -1.0, 1.0);
            double xt = Math.pow(gamepad1.right_trigger, 3) - Math.pow(gamepad1.left_trigger, 3);
            double x = -Range.clip(Math.pow(gamepad1.left_stick_x, 3) + xt, -1.0, 1.0) * 1.1;
            double rx = Range.clip(-Math.pow(gamepad1.right_stick_x, 3), -1.0, 1.0);

            // Read inverse IMU heading, as the UMG heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle + offsetHeading;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            motorFrontLeft.setPower(fastMode ? frontLeftPower : slowMode ? frontLeftPower /3 : frontLeftPower / 2);
            motorFrontRight.setPower(fastMode ?  frontRightPower : slowMode ? frontRightPower /3 : frontRightPower / 2);
            motorBackLeft.setPower(fastMode ? backLeftPower : slowMode ? backLeftPower /3 : backLeftPower / 2);
            motorBackRight.setPower(fastMode ? backRightPower : slowMode ? backRightPower /3 : backRightPower / 2 );


            driverGamePad.update();
            operatorGamePad.update();

            int liftMotor = motorLift.getCurrentPosition();
            String liftPosition = Integer.toString(liftMotor);
            String clawPosition = Double.toString(claw.getPosition());

            telemetry.addLine("Lift position:" + liftPosition);
            telemetry.addLine("Claw position" + clawPosition);





            telemetry.update();
        }
    }

    private void OnDriverGamePadChange(FtcGamePad ftcGamePad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:

                if(pressed)
                    fastMode = !fastMode;
                    // turns fast mode on
                break;

            case FtcGamePad.GAMEPAD_B:

                if(pressed)
                    slowMode = !slowMode;
                //turns fast mode off

                break;




            case FtcGamePad.GAMEPAD_BACK:
                if(pressed)
                    offsetHeading = imu.getAngularOrientation().firstAngle;
                break;






        }
    }

    private void OnOperatorGamePadChange(FtcGamePad ftcGamePad, int button, boolean pressed) {

        switch (button) {

            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                if(pressed)
                    //closing the claw
                    claw.setPosition(.40);
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                if(pressed)
                    //opening the claw
                    claw.setPosition(0);

                break;


            case FtcGamePad.GAMEPAD_DPAD_UP:
                motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               if(pressed)
                    motorLift.setPower(0.95);
               else
                   motorLift.setPower(0);
               break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(pressed && motorLift.getCurrentPosition() > 0)
                        motorLift.setPower(-.90);
                else
                    motorLift.setPower(0);
                break;


            case FtcGamePad.GAMEPAD_A:
                if(pressed){
                    // going to the ground
                    motorLift.setTargetPosition(0);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(0.9);
                    break;
                }


            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    // goes to the medium
                    motorLift.setTargetPosition(1415);
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLift.setPower(.90);
                    break;
                }

           case FtcGamePad.GAMEPAD_B:
               if(pressed){
                   motorLift.setTargetPosition(859);
                   motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   motorLift.setPower(.90);
                   break;
               }





















        }
        }
}




