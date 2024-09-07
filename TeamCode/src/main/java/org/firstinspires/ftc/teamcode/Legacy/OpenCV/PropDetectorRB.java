package org.firstinspires.ftc.teamcode.Legacy.OpenCV;//package org.firstinspires.ftc.teamcode.OpenCv;
//
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
////Declare Variables
//@Autonomous(name = "PropDetectorRB", group = "Auto", preselectTeleOp = "DoubleGamepad")
//public class PropDetectorRB extends LinearOpMode {
//    double TICKS_PER_INCH = 307.695;
//    double ROBOT_WIDTH = 13;
//    double MIN_POSITION = 0, MAX_POSITION = 1;
//    public void traverseY(double inY, double speed) {
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        FrontLeft.setPower(speed*inY/Math.abs(inY));
//        FrontRight.setPower(speed*inY/Math.abs(inY));
//        BackLeft.setPower(speed*inY/Math.abs(inY));
//        BackRight.setPower(speed*inY/Math.abs(inY));
//
//        if (inY >0) {
//            while (verticalRight.getCurrentPosition() <= inY*TICKS_PER_INCH){
//                telemetry.addData("verticalRightPosition",verticalRight.getCurrentPosition());
//                telemetry.addData("inY*ticks", inY*TICKS_PER_INCH);
//                telemetry.update();
//            }
//        } else {
//            while (verticalRight.getCurrentPosition() >= inY*TICKS_PER_INCH){
//
//                telemetry.addData("verticalRightPosition",verticalRight.getCurrentPosition());
//                //telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
//                //telemetry.addData("BackLeft Position", BackLeft.getCurrentPosition());
//                //telemetry.addData("FrontRight Position", FrontRight.getCurrentPosition());
//                //telemetry.addData("BackRight Position", BackRight.getCurrentPosition());
//                telemetry.update();
//                //idle();
//            }
//        }
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//        //telemetry.addData("verticalRightPosition",verticalRight.getCurrentPosition());
//        //telemetry.update();
//    }
//    public void traverseY_WithLoopLimit(double inY, double speed, int MAX_LOOPS) {
//        int loopCtr = 0;
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        FrontLeft.setPower(speed * inY / Math.abs(inY));
//        FrontRight.setPower(speed * inY / Math.abs(inY));
//        BackLeft.setPower(speed * inY / Math.abs(inY));
//        BackRight.setPower(speed * inY / Math.abs(inY));
//
//        if (inY > 0) {
//            while (verticalRight.getCurrentPosition() <= inY * TICKS_PER_INCH && loopCtr < MAX_LOOPS) {
//                loopCtr++;
//                telemetry.addData("Number of loops", loopCtr);
//                telemetry.update();
//            }
//        } else {
//            while (verticalRight.getCurrentPosition() >= inY * TICKS_PER_INCH && loopCtr < MAX_LOOPS) {
//                loopCtr++;
//                telemetry.addData("Number of loops", loopCtr);
//                telemetry.update();
//
//            }
//        }
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//    }
//
//
//    public void traverseX(double inX, double speed) {
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        double denominator = (speed*inX)/Math.abs(inX);
//        double rx = 0.01*inX/Math.abs(inX);
//        double y=0;
//        double x=0.5;
//        double frontLeftPower = (y + x + rx) / (1.95*denominator);
//        double backLeftPower = (y - x + rx) / (2*denominator);
//        double frontRightPower = (y - x - rx) / (2*denominator);
//        double backRightPower = (y + x - rx) / (2*denominator);
//
//
//        FrontLeft.setPower(frontLeftPower);
//        BackLeft.setPower(backLeftPower);
//        FrontRight.setPower(frontRightPower);
//        BackRight.setPower(backRightPower);
//
//
//        if (inX > 0) {
//            while (-1*horizontal.getCurrentPosition() <= inX*TICKS_PER_INCH){
//                telemetry.addData("horizontalEncoderPosition",horizontal.getCurrentPosition());
//                telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
//                telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
//                telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
//                telemetry.addData("BackRight",BackRight.getCurrentPosition());
//                telemetry.addData("Ticks to be travelled",inX*TICKS_PER_INCH);
//                telemetry.update();
//            }
//        }
//        else {
//            while (-1*horizontal.getCurrentPosition() >= inX*TICKS_PER_INCH){
//                telemetry.addData("horizontalEncoderPosition",horizontal.getCurrentPosition());
//                telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
//                telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
//                telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
//                telemetry.addData("BackRight",BackRight.getCurrentPosition());
//                telemetry.addData("Ticks to be travelled",inX*TICKS_PER_INCH);
//                telemetry.update();
//            }
//        }
//
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//        telemetry.addData("horizontalEncoderPosition",horizontal.getCurrentPosition());
//        telemetry.update();
//    }
//
//
//
//    public void turn(double TARGET_ANGLE, double power) {
//        imu.resetYaw();
//        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double TURN_ANGLE = TARGET_ANGLE;
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("target", "%.2f Deg. (Heading)", TURN_ANGLE);
//        telemetry.update();
//
//
//        if (TURN_ANGLE < 0) {
//
//            FrontLeft.setPower(power);
//            FrontRight.setPower(-power);
//            BackLeft.setPower(power);
//            BackRight.setPower(-power);
//
//            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > TURN_ANGLE) {
//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//                telemetry.update();
//            }
//        } else {
//            FrontLeft.setPower(-power);
//            FrontRight.setPower(power);
//            BackLeft.setPower(-power);
//            BackRight.setPower(power);
//
//            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < TURN_ANGLE) {
//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//                telemetry.update();
//            }
//        }
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.update();
//    }
//
//        public void turnHeading(double TARGET_ANGLE, double power) {
//
//        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double TURN_ANGLE = TARGET_ANGLE;
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.addData("target", "%.2f Deg. (Heading)", TURN_ANGLE);
//        telemetry.update();
//
//
//        if (TURN_ANGLE < 0) {
//
//            FrontLeft.setPower(power);
//            FrontRight.setPower(-power);
//            BackLeft.setPower(power);
//            BackRight.setPower(-power);
//
//            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > TURN_ANGLE) {
//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//                telemetry.update();
//            }
//        } else {
//            FrontLeft.setPower(-power);
//            FrontRight.setPower(power);
//            BackLeft.setPower(-power);
//            BackRight.setPower(power);
//
//            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < TURN_ANGLE) {
//                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//                telemetry.update();
//            }
//        }
//        FrontLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackLeft.setPower(0);
//        BackRight.setPower(0);
//
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        telemetry.update();
//    }
//
//
//
//
//    public void motorLock(double targetEncoderValue) {
//        // Set power level
//        try {
//            double powerLevel = 0.5; // Replace with your desired power level
//            ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            // Apply power until the motor reaches the target encoder value
//            while (ArmMotor.getCurrentPosition() < targetEncoderValue) {
//                telemetry.addData("Current position", ArmMotor.getCurrentPosition());
//                telemetry.update();
//                ArmMotor.setPower(powerLevel);
//            }
//            ArmMotor.setPower(0.01);
//            // Motor reached the target encoder value, set it to brake mode
//        } catch(Exception e) {
//            e.printStackTrace();
//            telemetry.addLine("motor lock crashed with " + e);
//            telemetry.update();
//        }
//    }
//
//    private DcMotor FrontLeft, BackLeft, FrontRight, BackRight;
//    private DcMotor ArmMotor;
//    private DcMotor verticalRight, horizontal;
//    private Servo leftGripperServo, rightGripperServo;
//    private BHI260IMU imu;
//
//    //private Telemetry telemetry;
//
//    OpenCvCamera camera;
//    PropDetectionPipelineRedHSV PropDetector;
//
//    public void RightBackboardZone1Movement() {
//        imu.resetYaw();
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        traverseY(27,0.4);
//        sleep(500);
//        turn(75, 0.3);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        traverseY(-1.5,0.2);
//        sleep(500);
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        traverseY(-10,0.4);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        traverseY(3,0.4);
//        sleep(500);
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        turn(-165,0.3);
//        sleep(500);
//        traverseY(20,0.4);
//        sleep(500);
//        traverseX(-7, 0.7);
//        sleep(500);
//        motorLock(400);
//        ArmMotor.setPower(-0.075);
//        traverseY_WithLoopLimit(14,0.25, 550);
//        ArmMotor.setPower(0.04);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        ArmMotor.setPower(0.5);
//        traverseY(-6,0.5);
//        ArmMotor.setPower(0);
//        sleep(500);
//        traverseX(27,0.4);
//        sleep(500);
//        traverseY(5, 0.7);
//        sleep(10000);
//    }
//    public void RightBackboardZone2Movement() {
//        imu.resetYaw();
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        traverseY(26.5,0.4);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        traverseY(-1.5,0.2);
//        sleep(500);
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        traverseY(-2,0.2);
//        sleep(500);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turn(-87,0.2);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        traverseY(31,0.4);
//        sleep(500);
//        leftGripperServo.setPosition(0.7);
//        rightGripperServo.setPosition(0.3);
//        sleep(500);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turnHeading(-93,0.2);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sleep(500);
//        traverseX(-4,0.8);
//        motorLock(380);
//        ArmMotor.setPower(-0.075);
//        traverseY_WithLoopLimit(11,0.25, 300);
//        ArmMotor.setPower(0.04);
//        sleep(500);
//        leftGripperServo.setPosition(0.3);
//        rightGripperServo.setPosition(0.7);
//        sleep(500);
//        ArmMotor.setPower(0.5);
//        traverseY(-2,0.5);
//        ArmMotor.setPower(0);
//        sleep(500);
//        traverseX(27,0.4);
//        sleep(500);
//        traverseY(5, 0.7);
//        sleep(10000);
//    }
//
//    public void RightBackboardZone3Movement() {
//            imu.resetYaw();
//            leftGripperServo.setPosition(0.7);
//            rightGripperServo.setPosition(0.3);
//            sleep(500);
//            traverseY(23,0.4);
//            sleep(500);
//            traverseX(11.5,0.7);
//            leftGripperServo.setPosition(0.3);
//            rightGripperServo.setPosition(0.7);
//            sleep(500);
//            traverseY(-1.5,0.2);
//            sleep(500);
//            leftGripperServo.setPosition(0.7);
//            rightGripperServo.setPosition(0.3);
//            sleep(500);
//            traverseY(-2,0.2);
//            sleep(500);
//            turn(-90,0.3);
//            sleep(500);
//            leftGripperServo.setPosition(0.3);
//            rightGripperServo.setPosition(0.7);
//            sleep(500);
//            traverseY(13,0.4);
//            sleep(500);
//            leftGripperServo.setPosition(0.7);
//            rightGripperServo.setPosition(0.3);
//            sleep(500);
//            traverseX(-3, 1);
//            sleep(500);
//            motorLock(380);
//            ArmMotor.setPower(-0.075);
//            traverseY_WithLoopLimit(9,0.25,300);
//            ArmMotor.setPower(0.04);
//            sleep(500);
//            leftGripperServo.setPosition(0.3);
//            rightGripperServo.setPosition(0.7);
//            sleep(500);
//            ArmMotor.setPower(0.5);
//            traverseY(-3,0.5);
//            ArmMotor.setPower(0);
//            sleep(500);
//            traverseX(15,0.4);
//            sleep(500);
//            traverseY(5, 0.7);
//            sleep(10000);
//    }
//
//    public void Init() {
//        imu = (BHI260IMU) hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
//        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
//        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
//        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
//        ArmMotor = hardwareMap.get(DcMotor.class,"armMotor");
//        leftGripperServo = hardwareMap.get(Servo.class,"leftGripperServo");
//        rightGripperServo = hardwareMap.get(Servo.class,"rightGripperServo");
//        verticalRight = hardwareMap.get(DcMotor.class,"verticalRight");
//        horizontal = hardwareMap.get(DcMotor.class,"horizontal");
//
//        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
//        BackLeft.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    static int opModeLoopCounter = 0;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Init();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera= OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        PropDetectionPipelineRedHSV  detector = new PropDetectionPipelineRedHSV(telemetry);
//        camera.setPipeline(detector);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                //telemetry.addLine("Starting streaming..");
//                camera.startStreaming(432,240, OpenCvCameraRotation.UPRIGHT); //800,448
//                //telemetry.addLine("..Done");
//                //telemetry.update();
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                //telemetry.addLine("Camera error" + errorCode);
//                //telemetry.update();
//
//            }
//        });
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addLine("opModeIsActive Loop counter: " + opModeLoopCounter++);
//            telemetry.addData("camera Frame Count", camera.getFrameCount());
//            telemetry.addData("camera FPS", String.format("%.2f", camera.getFps()));
//            telemetry.addData("camera Total frame time ms", camera.getTotalFrameTimeMs());
//            switch (detector.getAnalysis()) {
//                case CENTER:
//                    RightBackboardZone2Movement();
//                    camera.stopStreaming();
//                    break;
//                case LEFT:
//                    RightBackboardZone1Movement();
//                    camera.stopStreaming();
//                    break;
//                case RIGHT:
//                    RightBackboardZone3Movement();
//                    camera.stopStreaming();
//                    break;
//                case UNKNOWN:
//                    telemetry.addData("Prop Position :", "Unknown");
//                    break;
//                default:
//                    break;
//            }
//            sleep(100);
//        }
//    }
//}