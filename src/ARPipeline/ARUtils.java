package ARPipeline;

import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

public class ARUtils {

	
	public static Mat frameToMat(Frame frame) {
		Mat mat = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC3);
		for (int row = 0; row < frame.getHeight(); row++) {
			for (int col = 0; col < frame.getWidth(); col++) {
				
				byte GREY = 0;
				byte R = 0;
				byte G = 0;
				byte B = 0;
				
				if (frame.getR() == null) {
					GREY = frame.getGrey()[frame.getWidth() * row + col];
					R = GREY;
					G = GREY;
					B = GREY;
				} else {
					
					B = frame.getB()[frame.getWidth() * row + col];
					G = frame.getG()[frame.getWidth() * row + col];
					R = frame.getR()[frame.getWidth() * row + col];
				}
				
				byte [] rgb = { B, G, R };
				mat.put(row, col, rgb);
			}
		}
		return mat;
	}
	
	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, byte [] RGB) {
		List<KeyPoint> keypointList = keypoints.toList();
		for (KeyPoint keypoint: keypointList) {
			boxHighlight(frame, (int)keypoint.pt.x, (int)keypoint.pt.y, RGB);
		}
	}
	
	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints) {
		byte [] RGB = {0, 0, (byte)255};
		boxHighlight(frame, keypoints, RGB);
	}
	
	public static void boxHighlight(Frame frame, int x, int y, byte [] RGB) {
		int boxSize = 21;
		int thickness = 1;
		int offset = boxSize / 2;
		
		// for every pixel in the box area around the point
		for (int row = 0; row < boxSize; row++) {
			for (int col = 0; col < boxSize; col++) {
				int pixelPosX = col + x - offset;
				int pixelPosY = row + y - offset;
				
				// do we color it?
				// check for boundaries of image
				if (pixelPosX > 0 && pixelPosX < frame.getWidth() && pixelPosY > 0 && pixelPosY < frame.getHeight()) {
					
					// check for distance from barrier
					if (row <= thickness || row >= boxSize - thickness || col <= thickness || col >= boxSize - thickness) {
						
						// paint the pixel
						frame.getGrey()[frame.getWidth() * pixelPosY + pixelPosX] = (byte)((RGB[0] + RGB[1] + RGB[2]) / 3);
						
						if (frame.getR() != null) {
							frame.getR()[frame.getWidth() * pixelPosY + pixelPosX] = RGB[0];
							frame.getG()[frame.getWidth() * pixelPosY + pixelPosX] = RGB[1];
							frame.getB()[frame.getWidth() * pixelPosY + pixelPosX] = RGB[2];
						}
						
					}
					
				}
			}
		}
	}
	
}
