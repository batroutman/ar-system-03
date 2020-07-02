import org.opencv.videoio.VideoCapture;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import ARPipeline.*;
import entities.Camera;
import entities.Entity;
import renderEngine.Renderer;
import shaders.StaticShader;

public class ARBootstrapper {
	
	String SAMPLE_PATH = "src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";
	
	public ARBootstrapper() {
		
	}
	
	public void start() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, true);
		SingletonPoseBuffer spb = new SingletonPoseBuffer();
		OpenGLFrameBuffer oglfb = new OpenGLFrameBuffer();
		ARPipeline pipeline = new TestPipeline(ofb, spb, oglfb);
		
		pipeline.start();

		println("Done.");
	}
	
	public static void println(Object obj) {
		System.out.println(obj);
	}
	
	public static void main(String [] args) {
		ARBootstrapper arBootstrapper = new ARBootstrapper();
		arBootstrapper.start();
	}
}