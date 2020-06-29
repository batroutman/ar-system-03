package ARPipeline;

import java.io.ByteArrayInputStream;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.CvType;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;

// Frame buffer that outputs frames directly to a canvas
public class CanvasFrameBuffer implements FrameBuffer {

	protected Canvas canvas = new Canvas();
	
	public CanvasFrameBuffer() {
		
	}
	
	public CanvasFrameBuffer(Canvas canvas) {
		this.canvas = canvas;
	}
	
	public void setCanvas(Canvas canvas) {
		this.canvas = canvas;
	}
	
	public Canvas getCanvas() {
		return this.canvas;
	}
	
	public static Mat frameToMat(Frame frame) {
		Mat mat = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC3);
		for (int row = 0; row < frame.getHeight(); row++) {
			for (int col = 0; col < frame.getWidth(); col++) {
				
				byte Y = 0;
				byte U = 0;
				byte V = 0;
				
				byte R = 0;
				byte G = 0;
				byte B = 0;
				
				if (frame.getU() == null) {
					Y = frame.getY()[frame.getWidth() * row + col];
					R = Y;
					G = Y;
					B = Y;
				} else {
					Y = frame.getY()[frame.getWidth() * row + col];
					U = frame.getU()[frame.getWidth() * row + col];
					V = frame.getV()[frame.getWidth() * row + col];
					
					B = (byte) Math.floor(1.164 * (Y - 16) + 2.018 * (U - 128));
					G = (byte) Math.floor(1.164 * (Y - 16) - 0.813 * (V - 128) - 0.391 * (U - 128));
					R = (byte) Math.floor(1.164 * (Y - 16) + 1.596 * (V - 128));
				}
				
				byte [] rgb = { R, G, B };
				mat.put(row, col, rgb);
			}
		}
		return mat;
	}
	
	public void pushFrame(Frame frame) {
		
		// clear frame
		GraphicsContext gc = this.canvas.getGraphicsContext2D();
		
		// create image
		Mat mat = frameToMat(frame);
		MatOfByte byteMat = new MatOfByte();
		Imgcodecs.imencode(".bmp", mat, byteMat);
		Image img = new Image(new ByteArrayInputStream(byteMat.toArray()));
		
		// draw image
		gc.drawImage(img, 0, 0, frame.getWidth(), frame.getHeight(), 0, 0, this.canvas.getWidth(), this.canvas.getHeight());
		
	}
	
	public Frame getCurrentFrame() {
		return null;
	}
	
}
