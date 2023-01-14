// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {

  private Thread vision_thread;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    vision_thread = initializeVisionThread();
    vision_thread.setDaemon(true);
    vision_thread.start();
  }

  private Thread initializeVisionThread() {
    return new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();

      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getVideo(camera);
      CvSource outputStream = CameraServer.putVideo("Cone Mask", 640, 480);

      Mat img = new Mat();

      Scalar lowHsv = new Scalar(25, 150, 20);
      Scalar highHsv = new Scalar(37, 255, 255);

      while (!Thread.interrupted()) {
        if (Robot.isReal() && cvSink.grabFrame(img) == 0) {
          System.out.println("Camera error");
          continue;
        }
        else if (Robot.isSimulation()) {
          img = Imgcodecs.imread("./VisionExamples/Cone1.png");
        }
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        Core.inRange(img, lowHsv, highHsv, img);

        int pixelCount = 0;
        int ySum = 0;
        int xSum = 0;
        for (int y = 0; y < img.rows(); y++) {
          for (int x = 0; x < img.cols(); x++) {
            if (img.get(y, x)[0] == 255) {
              pixelCount++;
              ySum += y;
              xSum += x;
            }
          }
        }
        outputStream.putFrame(img);
        SmartDashboard.putNumber("Average X", xSum / (double) pixelCount);
        SmartDashboard.putNumber("Average Y", ySum / (double) pixelCount);
        }
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
