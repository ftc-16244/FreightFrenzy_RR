package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Enums.Barcode;
import org.firstinspires.ftc.teamcode.Enums.GameElement;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
public class RedCarouselAutoMeet2 extends LinearOpMode {

    public static double DISTANCE = 30; // in
    public ElapsedTime   tfTime      = new ElapsedTime(); // timer for tensor flow
    FelipeDeux felipe = new FelipeDeux(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();


    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.LOAD;

    Barcode barcode = Barcode.RIGHT; // Default target zone
    GameElement gameElement = GameElement.BALL; //set to Ball since this is impossible

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQXVmfz/////AAABmXaLleqhDEfavwYMzTtToIEdemv1X+0FZP6tlJRbxB40Cu6uDRNRyMR8yfBOmNoCPxVsl1mBgl7GKQppEQbdNI4tZLCARFsacECZkqph4VD5nho2qFN/DmvLA0e1xwz1oHBOYOyYzc14tKxatkLD0yFP7/3/s/XobsQ+3gknx1UIZO7YXHxGwSDgoU96VAhGGx+00A2wMn2UY6SGPl+oYgsE0avmlG4A4gOsc+lck55eAKZ2PwH7DyxYAtbRf5i4Hb12s7ypFoBxfyS400tDSNOUBg393Njakzcr4YqL6PYe760ZKmu78+8X4xTAYSrqFJQHaCiHt8HcTVLNl2fPQxh0wBmLvQJ/mvVfG495ER1A";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime timer = new ElapsedTime();
        double ducktime = 2.5; // carosel rotation time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.juanMechanicalReset();

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories Here
        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj0 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39,30,Math.toRadians(0)))
                //.addTemporalMarker(-.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory  traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(39,-6,Math.toRadians(0)))
                .addTemporalMarker(-.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();

        Trajectory  traj2 = drive.trajectoryBuilder(traj1.end())
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(39,29,Math.toRadians(90)))

                .build();

        Trajectory  traj2A = drive.trajectoryBuilder(traj2.end())

                .lineToLinearHeading(new Pose2d(11,29,Math.toRadians(90)))

                .build();

        Trajectory  traj3 = drive.trajectoryBuilder(traj2A.end())
                // final touch up to engage carousel
                .strafeLeft(6)
                .addTemporalMarker(.25,()->{carousel.carouselTurnCWAuto();})
                .build();
        Trajectory  traj4 = drive.trajectoryBuilder(traj3.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(30,29,Math.toRadians(-90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();
        Trajectory  traj5 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(41,3,Math.toRadians(179)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory  traj6 = drive.trajectoryBuilder(traj5.end())
                .addTemporalMarker(-0.6,()->{felipe.thumbOpen();})
                .addTemporalMarker(.1,()->{felipe.thumbClose();})
                .addTemporalMarker(.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(9,-30,Math.toRadians(-180)))

                .build();
        Trajectory  traj7 = drive.trajectoryBuilder(traj6.end())
                // final touch up to engage carousel
                .forward(5)
                .addTemporalMarker(.25,()->{carousel.carouselTurnCCW();})
                .build();
        Trajectory  traj8 = drive.trajectoryBuilder(traj7.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(26,-28,Math.toRadians(90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();




        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

        }

        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        tfTime.reset(); //  reset the TF timer
        // if (opModeIsActive()) {
        // Note the while loop below stays in the loop "forever" because there is no way to escape it.
        // change the argument to something like this  "while (opModeIsActive() && ftTime.seconds() < tfAllowedTime)"
        // that way you give tf a second or possibly 2 seconds to find the duck then move on with the rest of the code.
        // the sample opmode this came from only did tensor flow and it had to stay active all the time so you can see how it works. We have
        // to change that part when we put into an autonomous opmode that does other functions.
        while (opModeIsActive() && tfTime.seconds() < 2) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if (recognition.getLabel()=="Duck"){
                            gameElement = GameElement.DUCK;
                        }

                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if ( recognition.getLeft() >25 &&  recognition.getLeft() < 250 && gameElement == GameElement.DUCK){
                            barcode = Barcode.CENTER;}
                        else if (recognition.getLeft() >300 &&  recognition.getLeft() < 600 && gameElement == GameElement.DUCK){
                            barcode = Barcode.RIGHT;
                        }
                        i++;


                        telemetry.addData("Barcode with Duck",barcode);

                    }
                }
                telemetry.update();
            }

        }
        telemetry.addData("highgoal", barcode);
        if (tfod != null) {
            tfod.shutdown();
        }


        switch(barcode){
            case RIGHT: //
                felipe.liftRise();
                drive.followTrajectory(traj0);
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj2A);
                felipe.liftLoad();
                drive.followTrajectory(traj3);

                //delay to let carousel turn
                timer.reset();
                while(timer.seconds() < ducktime) drive.update();

                drive.followTrajectory(traj4);

                break;
                case CENTER: //
                    drive.followTrajectory(traj0);
                    drive.followTrajectory(traj1);
                    drive.followTrajectory(traj2);
                    drive.followTrajectory(traj2A);
                    drive.followTrajectory(traj3);

                //delay to let carousel turn
                timer.reset();
                while(timer.seconds() < ducktime) drive.update();

                drive.followTrajectory(traj4);

                break;
            case LEFT:
                drive.followTrajectory(traj5);
                drive.followTrajectory(traj6);
                felipe.liftLoad();
                drive.followTrajectory(traj7);

                //delay to let carousel turn
                timer.reset();
                while(timer.seconds() < ducktime) drive.update();

                drive.followTrajectory(traj8);

                break;


        }
        //}

        if (isStopRequested()) return;


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
