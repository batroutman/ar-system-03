package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;

import Jama.Matrix;

public class MockPipeline extends ARPipeline {

	MockPointData mock = new MockPointData();
	long frameNum = 0;

	Matrix K = new Matrix(3, 3);

	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;
	ArrayList<Correspondence> lastCorrespondences = null;

	Pose pose = new Pose();

	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public MockPipeline() {
		super();
		this.init();
	}

	public MockPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public MockPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.init();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public void init() {
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

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

		this.pose.setMatrix(R.get(0, 0), R.get(0, 1), R.get(0, 2), R.get(1, 0), R.get(1, 1), R.get(1, 2), R.get(2, 0),
				R.get(2, 1), R.get(2, 2), t.get(0, 0), t.get(1, 0), t.get(2, 0), System.nanoTime());
	}

	Mat oldDesc = new Mat();
	int count = 0;

	protected void mainloop() {
		boolean keepGoing = true;
		while (keepGoing) {

			if (this.frameNum >= this.mock.getMAX_FRAMES() - 1) {
				keepGoing = false;
				continue;
			}

			// find ORB features (omit for mock)

			// if no keyframes exist, generate one
			if (this.keyframes.size() == 0) {
				this.currentKeyFrame = new KeyFrame();
				this.currentKeyFrame.setPose(new Pose());
				this.currentKeyFrame.setKeypoints(this.mock.getKeypoints(this.frameNum));
				this.keyframes.add(this.currentKeyFrame);
			}
			// b. otherwise,
			else {

				// compute fundamental matrix -> essential matrix -> [ R t ]
				MatOfPoint2f keyframeMat = new MatOfPoint2f();
				MatOfPoint2f matKeypoints = new MatOfPoint2f();
				keyframeMat.fromList(this.currentKeyFrame.getKeypoints());
				matKeypoints.fromList(this.mock.getKeypoints(this.frameNum));
				Mat fundamentalMatrix = Calib3d.findFundamentalMat(keyframeMat, matKeypoints, Calib3d.FM_RANSAC, 3,
						0.99);

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

				EssentialDecomposition decomp = ARUtils.decomposeEssentialMat(eMatrix);

				ArrayList<Correspondence2D2D> correspondences = new ArrayList<Correspondence2D2D>();
				for (int i = 0; i < this.currentKeyFrame.getKeypoints().size(); i++) {
					Correspondence2D2D c = new Correspondence2D2D(this.currentKeyFrame.getKeypoints().get(i).x,
							this.currentKeyFrame.getKeypoints().get(i).y,
							this.mock.getKeypoints(this.frameNum).get(i).x,
							this.mock.getKeypoints(this.frameNum).get(i).y);
					correspondences.add(c);
				}

				Rt rt = ARUtils.selectEssentialSolution(decomp, this.pose.getHomogeneousMatrix(), correspondences);
				Matrix R = this.mock.getR(this.frameNum);
				Matrix c = this.mock.getIC(this.frameNum).getMatrix(0, 2, 3, 3);
				// c.print(5, 4);
				Matrix t = R.getMatrix(0, 2, 0, 2).times(c);

				this.updatePose(R, t);
				// this.updatePose(rt.getR(), rt.getT());

			}

			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}

			// for demo, just push the unaltered frame along to the output
			// buffer
			synchronized (this.outputFrameBuffer) {
				int width = 480;
				int height = 270;

				Frame currentFrame = ARUtils.artificialKeypointFrame(this.mock.getKeypoints(this.frameNum), width,
						height);
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			this.frameNum++;

			try {
				Thread.sleep(10);
			} catch (Exception e) {

			}
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