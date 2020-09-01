package ARPipeline;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
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

		Matrix4f updatedPose = new Matrix4f();
		updatedPose.setIdentity();
		updatedPose.m00 = (float) R.get(0, 0);
		updatedPose.m01 = (float) R.get(0, 1);
		updatedPose.m02 = (float) R.get(0, 2);
		updatedPose.m10 = (float) R.get(1, 0);
		updatedPose.m11 = (float) R.get(1, 1);
		updatedPose.m12 = (float) R.get(1, 2);
		updatedPose.m20 = (float) R.get(2, 0);
		updatedPose.m21 = (float) R.get(2, 1);
		updatedPose.m22 = (float) R.get(2, 2);
		updatedPose.translate(new Vector3f((float) t.get(0, 0), (float) t.get(1, 0), (float) t.get(2, 0)));

		Matrix4f.mul(updatedPose, this.currentKeyFrame.getPose().getHomogeneousMatrix4f(), updatedPose);
		this.pose.setMatrix(updatedPose);
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
				Matrix4f R4 = new Matrix4f();
				R4.setIdentity();
				float rotX = this.frameNum * 0.000f;
				float rotY = this.frameNum * 0.000f;
				float rotZ = this.frameNum * 0.000f;
				R4.rotate(rotX, new Vector3f(1, 0, 0));
				R4.rotate(rotY, new Vector3f(0, 1, 0));
				R4.rotate(rotZ, new Vector3f(0, 0, 1));
				Matrix R = ARUtils.Matrix4fToMatrix(R4).getMatrix(0, 2, 0, 2);

				Matrix c = new Matrix(3, 1);
				float dx = this.frameNum * 5f;
				float dy = this.frameNum * 5f;
				float dz = this.frameNum * 0.0f;
				c.set(0, 0, dx);
				c.set(1, 0, dy);
				c.set(2, 0, dz);

				Matrix t = R.times(c);

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
				Thread.sleep(100);
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