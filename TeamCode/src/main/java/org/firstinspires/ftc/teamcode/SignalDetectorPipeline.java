package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SignalDetectorPipeline extends OpenCvPipeline {
    Mat copy = new Mat();
    double redTotal;
    double greenTotal;
    double blueTotal;

    public int position = 0;

    public SignalDetectorPipeline(){
        //lol
    }

    public Mat processFrame(Mat input){
        input.copyTo(copy);

        if(copy.empty()){
            return input;
        }

        Imgproc.rectangle(copy, new Rect(50,50,50,50), new Scalar(0,255,0));
        Mat subMat = copy.submat(new Rect(50, 50, 50, 50));

        redTotal = Core.sumElems(subMat).val[0];
        greenTotal = Core.sumElems(subMat).val[1];
        blueTotal = Core.sumElems(subMat).val[2];

        double max = Math.max(redTotal, Math.max(greenTotal, blueTotal));

        if(max == redTotal){
            position = 0;
        }
        if(max == greenTotal){
            position = 1;
        }
        if(max == blueTotal){
            position = 2;
        }

        subMat.release();
        //copy.release();
        return copy;
    }
}
