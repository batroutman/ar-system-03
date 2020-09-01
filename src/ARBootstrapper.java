import org.opencv.core.Core;

import ARPipeline.ARPipeline;
import ARPipeline.CameraIntrinsics;
import ARPipeline.MockPipeline;
import ARPipeline.MockPointData;
import ARPipeline.SingletonFrameBuffer;
import ARPipeline.SingletonPoseBuffer;
import Jama.Matrix;

public class ARBootstrapper {

	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";

	public ARBootstrapper() {

	}

	public void start() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, false);
		SingletonPoseBuffer spb = new SingletonPoseBuffer();
		SingletonFrameBuffer sfb = new SingletonFrameBuffer();
		// ARPipeline pipeline = new TestPipeline(ofb, spb, sfb);
		ARPipeline pipeline = new MockPipeline(sfb, spb, sfb);
		OpenGLARDisplay ARDisplay = new OpenGLARDisplay(sfb, spb);

		pipeline.start();
		ARDisplay.displayLoop();

		println("Done.");
	}

	public static void println(Object obj) {
		System.out.println(obj);
	}

	public static void main(String[] args) {
		ARBootstrapper arBootstrapper = new ARBootstrapper();
		arBootstrapper.start();
		// arBootstrapper.tests();
	}

	public void tests() {
		Matrix K = new Matrix(3, 3);
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);

		Matrix worldPoint = new Matrix(4, 1);
		worldPoint.set(0, 0, 0);
		worldPoint.set(1, 0, 0);
		worldPoint.set(2, 0, 1000);
		worldPoint.set(3, 0, 1);

		Matrix Rx = Matrix.identity(4, 4);
		Rx.set(1, 1, Math.cos(Math.PI));
		Rx.set(2, 2, Math.cos(Math.PI));
		Rx.set(1, 2, -Math.sin(Math.PI));
		Rx.set(2, 1, Math.sin(Math.PI));

		long frameNumber = 50;
		MockPointData mock = new MockPointData();
		Matrix R = mock.getR(frameNumber);
		Matrix IC = mock.getIC(frameNumber);

		Matrix viewMatrix = R.times(IC).times(Rx);

	}
}