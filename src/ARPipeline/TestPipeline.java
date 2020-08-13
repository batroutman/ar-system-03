package ARPipeline;

import org.opencv.core.Core;

import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.features2d.ORB;

import Jama.Matrix;
import Jama.SingularValueDecomposition;

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
	

	Matrix K = new Matrix(3, 3);
	
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
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0,  1, CameraIntrinsics.s);
		K.set(0,  2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2,  0, 0.0);
		K.set(2,  1, 0.0);
		K.set(2,  2, 1.0);
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
	
	public void updatePose(Matrix R, Matrix t) {
		
		Matrix4f updatedPose = new Matrix4f();
		updatedPose.setIdentity();
		updatedPose.m00 = (float)R.get(0, 0);
		updatedPose.m01 = (float)R.get(0, 1);
		updatedPose.m02 = (float)R.get(0, 2);
		updatedPose.m10 = (float)R.get(1, 0);
		updatedPose.m11 = (float)R.get(1, 1);
		updatedPose.m12 = (float)R.get(1, 2);
		updatedPose.m20 = (float)R.get(2, 0);
		updatedPose.m21 = (float)R.get(2, 1);
		updatedPose.m22 = (float)R.get(2, 2);
//		updatedPose.translate(new Vector3f((float)t.get(0, 0), (float)t.get(1, 0), (float)t.get(2, 0)));

		Matrix4f.mul(updatedPose, this.pose.getHomogeneous(), updatedPose);
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

//						Mat essentialMatrix = Mat.zeros(3, 3, CvType.CV_32F);
//						Core.gemm(this.K.t(), fundamentalMatrix, 1, Mat.zeros(3, 3, CvType.CV_32F), 0, essentialMatrix);
//						Core.gemm(essentialMatrix, this.K, 1, Mat.zeros(3, 3, CvType.CV_32F), 0, essentialMatrix);
//						
//						Mat R1 = Mat.zeros(3, 3, CvType.CV_32F);
//						Mat R2 = Mat.zeros(3, 3, CvType.CV_32F);
//						Mat t = Mat.zeros(3, 1, CvType.CV_32F);
//						Calib3d.decomposeEssentialMat(essentialMatrix, R1, R2, t);
						
						Matrix funMat = new Matrix(3, 3);
						funMat.set(0, 0, fundamentalMatrix.get(0, 0)[0]);
						funMat.set(0, 1, fundamentalMatrix.get(0, 1)[0]);
						funMat.set(0, 2, fundamentalMatrix.get(0, 2)[0]);
						funMat.set(1, 0, fundamentalMatrix.get(1, 0)[0]);
						funMat.set(1, 1, fundamentalMatrix.get(1, 1)[0]);
						funMat.set(1, 2, fundamentalMatrix.get(1, 2)[0]);
						funMat.set(2, 0, fundamentalMatrix.get(2, 0)[0]);
						funMat.set(2, 1, fundamentalMatrix.get(2, 1)[0]);
						funMat.set(2, 2, fundamentalMatrix.get(2, 2)[0]);
						
						Matrix eMatrix = this.K.transpose().times(funMat).times(this.K);
						
						System.out.println("[");
						System.out.println(fundamentalMatrix.get(0, 0)[0] + ", " + fundamentalMatrix.get(0, 1)[0] + ", " + fundamentalMatrix.get(0, 2)[0]);
						System.out.println(fundamentalMatrix.get(1, 0)[0] + ", " + fundamentalMatrix.get(1, 1)[0] + ", " + fundamentalMatrix.get(1, 2)[0]);
						System.out.println(fundamentalMatrix.get(2, 0)[0] + ", " + fundamentalMatrix.get(2, 1)[0] + ", " + fundamentalMatrix.get(2, 2)[0]);
						System.out.println("]");
						
						System.out.println("[");
						System.out.println(eMatrix.get(0, 0) + ", " + eMatrix.get(0, 1) + ", " + eMatrix.get(0, 2));
						System.out.println(eMatrix.get(1, 0) + ", " + eMatrix.get(1, 1) + ", " + eMatrix.get(1, 2));
						System.out.println(eMatrix.get(2, 0) + ", " + eMatrix.get(2, 1) + ", " + eMatrix.get(2, 2));
						System.out.println("]");
						
						System.out.println("About to calculate SVD...");
						SingularValueDecomposition svd = eMatrix.svd();
						System.out.println("SVD Calculated.");
						
						Matrix W = new Matrix(3, 3);
						W.set(0, 1, -1);
						W.set(1, 0, 1);
						W.set(2, 2, 1);
						
						Matrix Z = new Matrix(3, 3);
						Z.set(0, 1, 1);
						Z.set(1, 0, -1);
						
						Matrix mt = svd.getU().times(Z).times(svd.getU().transpose());
						Matrix t = new Matrix(3, 1);
						t.set(0, 0, mt.get(2, 1));
						t.set(1, 0, mt.get(0, 2));
						t.set(2, 0, mt.get(1, 0));
						Matrix R = svd.getU().times(W.transpose()).times(svd.getV().transpose());
						
						this.updatePose(R, t);
						

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