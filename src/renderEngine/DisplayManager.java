package renderEngine;

import org.lwjgl.LWJGLException;
import org.lwjgl.opengl.ContextAttribs;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.PixelFormat;

public class DisplayManager {

	public static int WIDTH = 1280;
	public static int HEIGHT = 960;
	private static final int FPS_CAP = 1000;

	public static void createDisplay(int width, int height) {

		WIDTH = width;
		HEIGHT = height;

		ContextAttribs attribs = new ContextAttribs(3, 2).withForwardCompatible(true).withProfileCore(true);

		try {
			Display.setDisplayMode(new DisplayMode(WIDTH, HEIGHT));
			Display.create(new PixelFormat(), attribs);
			Display.setTitle("Augmented Reality Testbed");
		} catch (LWJGLException e) {
			e.printStackTrace();
		}

		GL11.glViewport(0, 0, WIDTH, HEIGHT);
	}

	public static void updateDisplay() {

		Display.sync(FPS_CAP);
		Display.update();

	}

	public static void closeDisplay() {

		Display.destroy();

	}

}
