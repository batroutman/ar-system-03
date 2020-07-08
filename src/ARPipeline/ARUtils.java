package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;

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
	
	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, byte [][] RGB, int boxSize) {
		List<KeyPoint> keypointList = keypoints.toList();
		for (KeyPoint keypoint: keypointList) {
			if (true) { boxHighlight(frame, keypoint, RGB, boxSize * (int)Math.pow(2, keypoint.octave)); }
		}
	}
	
	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints) {
		byte [][] RGB = {{0, 0, (byte)255}, {(byte)255, 0, 0}, {0, (byte)255, 0}, {(byte)255, 0, (byte)255}};
		boxHighlight(frame, keypoints, RGB, 21);
	}
	
	public static void boxHighlight(Frame frame, MatOfKeyPoint keypoints, int boxSize) {
		byte [][] RGB = {{0, 0, (byte)255}, {(byte)255, 0, 0}, {0, (byte)255, 0}, {(byte)255, 0, (byte)255}};
		boxHighlight(frame, keypoints, RGB, boxSize);
	}
	
	public static void boxHighlight(Frame frame, KeyPoint keypoint, byte [][] RGB, int boxSize) {
		int thickness = 1;
		int offset = boxSize / 2;
		int x = (int)keypoint.pt.x;
		int y = (int)keypoint.pt.y;
		byte [] color = getRGB(RGB, keypoint.octave);
		
		// for every pixel in the box area around the point
		for (int row = 0; row < boxSize; row++) {
			for (int col = 0; col < boxSize; col++) {
				int pixelPosX = col + x - offset;
				int pixelPosY = row + y - offset;
				
				// do we color it?
				// check for boundaries of image
				if (pixelPosX > 0 && pixelPosX < frame.getWidth() && pixelPosY > 0 && pixelPosY < frame.getHeight()) {
					
					// check for distance from barrier
					if (row < thickness || row >= boxSize - thickness || col < thickness || col >= boxSize - thickness) {
						
						// paint the pixel
						frame.getGrey()[frame.getWidth() * pixelPosY + pixelPosX] = (byte)((color[0] + color[1] + color[2]) / 3);
						
						if (frame.getR() != null) {
							frame.getR()[frame.getWidth() * pixelPosY + pixelPosX] = color[0];
							frame.getG()[frame.getWidth() * pixelPosY + pixelPosX] = color[1];
							frame.getB()[frame.getWidth() * pixelPosY + pixelPosX] = color[2];
						}
						
					}
					
				}
			}
		}
	}
	
	public static byte[] getRGB(byte [][] RGB, int octave) {
		if (octave >= RGB.length) {
			return RGB[RGB.length - 1];
		} else {
			return RGB[octave];
		}
	}
	
	public static void nonMaximumSuppression(MatOfKeyPoint keypoints, double patchSize) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			// find all keypoints sufficiently overlapping this one
			List<KeyPoint> interestPoints = new ArrayList<KeyPoint>();
			interestPoints.add(keypointList.get(i));
			
			for (int j = i + 1; j < keypointList.size(); j++) {
				if (keypointList.get(i).octave == keypointList.get(j).octave && getDistance(keypointList.get(j).pt, keypointList.get(i).pt) < patchSize * Math.pow(2, keypointList.get(j).octave)) {
					interestPoints.add(keypointList.get(j));
				}
			}
			
			if (interestPoints.size() > 1) {
				// remove all interest points
				for (int k = 1; k < interestPoints.size(); k++) {
					keypointList.remove(interestPoints.get(k));
				}
					
				// insert centered point
				keypointList.set(i, getStrongest(interestPoints));
			}
			
		}
		keypoints.fromList(keypointList);
	}
	
	public static double getDistance(Point a, Point b) {
		double x = a.x - b.x;
		double y = a.y - b.y;
		return Math.sqrt(x * x + y * y);
	}
	
	public static KeyPoint getCentroid(List<KeyPoint> keypoints) {
		float sumX = 0;
		float sumY = 0;
		for (KeyPoint keypoint:keypoints) {
			sumX += keypoint.pt.x;
			sumY += keypoint.pt.y;
		}
		sumX = sumX / keypoints.size();
		sumY = sumY / keypoints.size();
		KeyPoint centered = new KeyPoint(sumX, sumY, keypoints.get(0).size, keypoints.get(0).angle, keypoints.get(0).response, keypoints.get(0).octave, keypoints.get(0).class_id);
		return centered;
	}
	
	public static KeyPoint getStrongest(List<KeyPoint> keypoints) {
		int strongestIndex = 0;
		for (int i = 0; i < keypoints.size(); i++) {
			if (keypoints.get(strongestIndex).response < keypoints.get(i).response) {
				strongestIndex = i;
			}
		}
		return keypoints.get(strongestIndex);
	}
	
	public static void pruneOctaves(MatOfKeyPoint keypoints, List<Integer> octavesToKeep) {
		List<KeyPoint> keypointList = new ArrayList<KeyPoint>(keypoints.toList());
		for (int i = 0; i < keypointList.size(); i++) {
			if (!octavesToKeep.contains(keypointList.get(i).octave)) {
				keypointList.remove(i);
				i--;
			}
		}
		keypoints.fromList(keypointList);
	}
	
	public static Double min(ArrayList<Double> list) {
		Double min = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (min > list.get(i)) {
				min = list.get(i);
			}
		}
		return min;
	}
	
	public static Double max(ArrayList<Double> list) {
		Double max = list.get(0);
		for (int i = 1; i < list.size(); i++) {
			if (max < list.get(i)) {
				max = list.get(i);
			}
		}
		return max;
	}
	
	public static Double sum(ArrayList<Double> list) {
		Double sum = 0.0;
		for (int i = 0; i < list.size(); i++) {
			sum += list.get(i);
		}
		return sum;
	}
	
	public static Double mean(ArrayList<Double> list) {
		return sum(list) / list.size();
	}
	
	public static Double variance(ArrayList<Double> list) {
		Double avg = mean(list);
		Double variance = 0.0;
		for (int i = 0; i < list.size(); i++) {
			variance += Math.pow(list.get(i) - avg, 2);
		}
		return variance / avg;
	}
	
	public static Double stdDev(ArrayList<Double> list) {
		return Math.sqrt(variance(list));
	}
}
