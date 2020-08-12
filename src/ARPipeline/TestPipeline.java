package ARPipeline;

import org.opencv.core.Core;

import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.features2d.ORB;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;

public class TestPipeline extends ARPipeline{
	
	double TOLERANCE = 150.0;
	double SEARCH_BOX_WIDTH = 60.0;
	

	Mat K = Mat.zeros(3, 3, CvType.CV_32F);
	
	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;
	ArrayList<Correspondence> lastCorrespondences = null;
	
	float rotAngle = 0;
	float translation = 0;
	Pose pose = new Pose();
	
	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public TestPipeline(){
		super();
		this.init();
	}

	public TestPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}
	
	public TestPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}
	
	public void init() {
		K.put(0, 0, CameraIntrinsics.fx);
		K.put(0,  1, CameraIntrinsics.s);
		K.put(0,  2, CameraIntrinsics.cx);
		K.put(1, 0, 0.0);
		K.put(1, 1, CameraIntrinsics.fy);
		K.put(1, 2, CameraIntrinsics.cy);
		K.put(2,  0, 0.0);
		K.put(2,  1, 0.0);
		K.put(2,  2, 1.0);
	}

	public void start() {
		this.mainThread.start();
	}
	public void stop() {
		this.mainThread.interrupt();
	}
	
	public static KeyFrame generateKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {
		
		KeyFrame keyframe = new KeyFrame();
		Pose pose = new Pose();
		pose.setOrigin();
		keyframe.setPose(pose);
		keyframe.setDescriptors(descriptors);
		keyframe.setKeypoints(keypoints);
		
		ArrayList<MapPoint> mapPoints = new ArrayList<MapPoint>();
		List<KeyPoint> keypointList = keypoints.toList();
		// may have issues here. negative y? offset?

		for (int i = 0; i < keypointList.size(); i++) {
			MapPoint mp = new MapPoint(descriptors.row(i));
			mp.setUV(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			mapPoints.add(mp);
		}
		
		keyframe.setMapPoints(mapPoints);
		return keyframe;
		
	}
	
	public void updatePose(Mat R1, Mat R2, Mat t) {
		
		Matrix4f updatedPose = new Matrix4f();
		updatedPose.setIdentity();
		boolean noNaN = t.get(0, 0)[0] == t.get(0, 0)[0] && t.get(1, 0)[0] == t.get(1, 0)[0] && t.get(2, 0)[0] == t.get(2, 0)[0];
		if (noNaN) {
			updatedPose.translate(new Vector3f((float)t.get(0, 0)[0], (float)t.get(1, 0)[0], (float)t.get(2, 0)[0]));
		}
		Matrix4f.mul(updatedPose, this.pose.getHomogeneous(), updatedPose);
		System.out.println(updatedPose.toString());
		this.pose.setMatrix(updatedPose);
	}
	
	
	Mat oldDesc = new Mat();
	int count  = 0;
	protected void mainloop() {
		Frame currentFrame = this.inputFrameBuffer.getCurrentFrame();
		boolean keepGoing = true;
		while (keepGoing) {
			
			if (currentFrame == null) {
				keepGoing = false;
				
				continue;
			}
			
			// find ORB features
			int patchSize = 5;
			ORB orb = ORB.create(100);
			orb.setScoreType(ORB.FAST_SCORE);
			orb.setPatchSize(patchSize);
			orb.setNLevels(1);
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			Mat descriptors = new Mat();
			Mat image = ARUtils.frameToMat(currentFrame);
			orb.detectAndCompute(image, new Mat(), keypoints, descriptors);
			descriptors.convertTo(descriptors, CvType.CV_32F);
			ARUtils.boxHighlight(currentFrame, keypoints, patchSize);
			oldDesc = descriptors;
			
			// convert keypoints to good format for fundamental mat
			List<Point> points = new ArrayList<Point>();
			List<KeyPoint> listKeypoints = keypoints.toList();
			for (int i = 0; i < Math.min(100, listKeypoints.size()); i++) {
				points.add(listKeypoints.get(i).pt);
			}
			
			// what do I want to do?
			// find correspondences between descriptors and mappoints in used in current keyframe
			//		a. if I have 0 keyframes, generate one and set pose to origin. continue.
					if (this.keyframes.size() == 0) {
						this.currentKeyFrame = generateKeyFrame(descriptors, keypoints);
						this.keyframes.add(this.currentKeyFrame);
					}
			//		b. otherwise, 
					else {
						
						// compute fundamental matrix -> essential matrix -> [ R t ]
						int min = Math.min(this.currentKeyFrame.getKeypoints().size(), listKeypoints.size());
						MatOfPoint2f keyframeMat = new MatOfPoint2f();
						MatOfPoint2f matKeypoints = new MatOfPoint2f();
						keyframeMat.fromList(this.currentKeyFrame.getKeypoints().subList(0, min));
						matKeypoints.fromList(points.subList(0,  min));
						Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_RANSAC, 10, 0.8);

						Mat essentialMatrix = Mat.zeros(3, 3, CvType.CV_32F);
						Core.gemm(this.K, fundamentalMatrix, 1, Mat.zeros(3, 3, CvType.CV_32F), 0, essentialMatrix);
						Core.gemm(essentialMatrix, this.K, 1, Mat.zeros(3, 3, CvType.CV_32F), 0, essentialMatrix);
						Mat R1 = Mat.zeros(3, 3, CvType.CV_32F);
						Mat R2 = Mat.zeros(3, 3, CvType.CV_32F);
						Mat t = Mat.zeros(3, 1, CvType.CV_32F);
						Calib3d.decomposeEssentialMat(essentialMatrix, R1, R2, t);
						this.updatePose(R1, R2, t);
						

			//			i. else (I don't have at least 50(?) correspondences), but I have at least 10, then generate new keyframe and use the same correspondences for fundamental matrix -> essential matrix -> [ R t ]. set current keyframe to new keyframe. continue.

			//			ii. else (<10 correspondences), check other keyframes for keyframe with most correspondences. If >= 10 correspondences, set as current keyframe and use the correspondences for sfm. if < 40, generate new keyframe.
			// 			iii. else, tracking lost (handle this later)
					}
			
			
			// rotate cube as demo that pose can be modified and displayed
//			rotAngle += 0.002f;
//			pose.setR11((float)Math.cos(rotAngle));
//			pose.setR22((float)Math.cos(rotAngle));
//			pose.setR12(-(float)Math.sin(rotAngle));
//			pose.setR21((float)Math.sin(rotAngle));
//			
			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}
			
			// for demo, just push the unaltered frame along to the output buffer
			synchronized (this.outputFrameBuffer) {
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			currentFrame = this.inputFrameBuffer.getCurrentFrame();
			
		}
	}
	
	public void getStats(Mat desc) {
		for (int i = 0; i < desc.cols(); i++) {
			ArrayList<Double> column = new ArrayList<Double>();
			for (int j = 0; j < desc.rows(); j++) {
				column.add(desc.get(j, i)[0]);
			}
			System.out.println("====  " + i + "  ===============================");
			System.out.println("Min    : " + ARUtils.min(column));
			System.out.println("Max    : " + ARUtils.max(column));
			System.out.println("Average: " + ARUtils.mean(column));
			System.out.println("Std Dev: " + ARUtils.stdDev(column));
		}
	}
	
	

}