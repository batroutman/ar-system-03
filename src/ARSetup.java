import org.opencv.videoio.VideoCapture;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import ARPipeline.*;
import entities.Camera;
import entities.Entity;
import renderEngine.Renderer;
import shaders.StaticShader;

public class ARSetup {
	
	String SAMPLE_PATH = "E:/Primary/Files/College/Research/ar-system-03/src/samples/";
	String filename = SAMPLE_PATH + "roomFloor01_270.avi";
	
	public ARSetup() {
		
	}
	
	
	public void start() {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		OfflineFrameBuffer ofb = new OfflineFrameBuffer(filename, true);
		OpenGLFrameBuffer oglfb = new OpenGLFrameBuffer();
		ARPipeline pipeline = new TestPipeline(ofb, oglfb);
		pipeline.start();

		println("Done.");
	}
	
	public static void println(Object obj) {
		System.out.println(obj);
	}
}