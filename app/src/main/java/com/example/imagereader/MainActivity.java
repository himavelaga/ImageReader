package com.example.imagereader;

import android.Manifest;
import android.annotation.TargetApi;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PorterDuff;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.RggbChannelVector;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Range;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View.OnClickListener;
import android.graphics.Bitmap;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.UUID;
import static android.os.Build.VERSION_CODES.KITKAT;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.CvType.CV_8UC3;
import static org.opencv.core.CvType.CV_32SC2;

/**
 * Via Camera, this application uses image processing techniques to detect the RGB of an object
 * @author Hima Velaga Zhuo Zhao
 */
@RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
public class MainActivity extends AppCompatActivity implements AdapterView.OnItemClickListener{

    private static final String TAG = "MainActivity";

    //camera initializations
    CaptureRequest.Builder captureRequestBuilder;
    CameraCaptureSession cameraSession;
    SurfaceView surfaceView;
    SurfaceHolder surfaceHolder;
    TextureView textureView;
    CameraDevice cameraDevice;
    String cameraId;
    android.util.Size imageDimensions;
    Handler backgroundHandler;
    HandlerThread handlerThread;
    CameraManager cameraManager;

    //Bluetooth initializations
    BluetoothAdapter mBluetoothAdapter;
    BluetoothConnectionService mBluetoothConnection;
    BluetoothDevice mBTDevice;
    Button btnEnableDisable_Discoverable;
    Button btnStartConnection;
    Button btnSend;
    TextView incomingMessages;
    StringBuilder messages;
    private static final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    public ArrayList<BluetoothDevice> mBTDevices = new ArrayList<>();
    public DeviceListAdapter mDeviceListAdapter;
    ListView lvNewDevices;
    int flag=1;
    EditText etSend;

    //int a=1;
    Bitmap bmp;
    Bitmap mask;
    Bitmap c;
    Bitmap meanMask;
    Bitmap meanConvexMask;
    Bitmap bitmap_get;
    List surfaces;

    //Variables to store RGB and HSV values
    int v1;
    int v2;
    int v3;
    static List<String> strings = new ArrayList<>();
    static int peanuts = 0;
    float meanr = 25;
    float meang = 125;
    float  meanb = 75;
    float hue = 100;
    float sat;
    float val;
    float h;
    float s;
    float v;
    String rgb;
    boolean startProcessing = false;
    boolean stopLoop = false;

    Button next;
    TextView h1;
    TextView pcolorPixelsText ;
    TextView pCountText ;
    TextView categoryText;
    TextView hsvView;
    int backgroundPixels;
    int width, height;
    Paint mpaint;
    Canvas canvas;
    Rect rec;
    ImageView iv;
    ImageReader mImageReader;
    Image image = null;

    //Opencv libarary variables
    Mat mRGB;
    Mat croppedMat;
    Mat convex;
    Mat convexMask;
    Mat hsvImg;
    Mat gray;
    Mat filter;
    Mat edges;
    Mat masked;
    Mat src;

    int prevColor = 0; // 0 if
    int currentColor = 0;
    int[] colors = new int[7]; //red, yellow, brown, black, green, orange

    //yellow, orangle, brown1, brown2, black RGB values
    int[][] categories = {{30, 200, 150},
            {20, 200, 90},
            {20, 200, 50},
            {20, 200, 50},
            {33, 21, 20},

    };


    /**
     * Sets up camera UI, bluetooth buttons
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //Surfaceview to display the camera
        surfaceView =(SurfaceView)findViewById(R.id.surfaveView);
        surfaceView.setZOrderOnTop(true);
        surfaceView.getHolder().setFormat(PixelFormat.TRANSPARENT);
        surfaceHolder=surfaceView.getHolder();

        //Paint and canvas creation for the rectangle field of view
        mpaint = new Paint();
        mpaint.setColor(Color.RED);
        mpaint.setStyle(Paint.Style.STROKE);
        mpaint.setStrokeWidth(2f);
        canvas = new Canvas();

        Button rectButton  = (Button) findViewById(R.id.rectangle);
        rectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                drawArea(); // DRAW RECTANGLE FIELD OF VIEW
            }
        });

        textureView = (TextureView) findViewById(R.id.texture);
        textureView.setSurfaceTextureListener(surfaceTextureListener);

        //***********************************Bluetooth buttons**************************************************
        Button btnONOFF = (Button) findViewById(R.id.btnONOFF);
        btnEnableDisable_Discoverable = (Button) findViewById(R.id.btnDiscoverable_on_off);
        lvNewDevices = (ListView) findViewById(R.id.lvNewDevices);
        //lvNewDevices.setVisibility(View.INVISIBLE);
        mBTDevices = new ArrayList<>();
        btnStartConnection = (Button) findViewById(R.id.btnStartConnection);
        btnSend = (Button) findViewById(R.id.btnSend);
        etSend = (EditText) findViewById(R.id.editText);
        incomingMessages = (TextView) findViewById(R.id.incomingMessage);
        messages = new StringBuilder();
        LocalBroadcastManager.getInstance(this).registerReceiver(mReceiver, new IntentFilter("incomingMessage"));
        //Broadcasts when bond state changes (ie:pairing)
        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        registerReceiver(mBroadcastReceiver4, filter);
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        lvNewDevices.setOnItemClickListener(MainActivity.this);


        btnONOFF.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(TAG, "onClick: enabling/disabling bluetooth.");
                enableDisableBT();
            }
        });

        btnStartConnection.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startConnection();
                lvNewDevices.setVisibility(View.GONE);
            }
        });

        btnSend.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                byte[] bytes = etSend.getText().toString().getBytes(Charset.defaultCharset());
                mBluetoothConnection.write(bytes);
                etSend.setText("");
            }
        });

    }

    /**
     * Processes frame, segments object and retrieves the color
     * @throws IOException
     */
    private void initializeOpenCVDependencies() throws IOException {
        src = new Mat();
        croppedMat.copyTo(src);
        long startTime = System.nanoTime();

        ImageProcessing processor = new ImageProcessing(); //utility class that contains image processing functions

        //Convert to HSV
         hsvImg = new Mat();
         Imgproc.cvtColor(src, hsvImg, Imgproc.COLOR_RGB2HSV);

        //*******************************Getting the number of background green color pixels********************************************
        Mat dst = new Mat();
        Core.inRange(hsvImg, new Scalar(35, 50, 50), new Scalar(85, 255, 255), dst);
        backgroundPixels = Core.countNonZero(dst);
        System.out.println("Number of green pixels: " + backgroundPixels);

        runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    // Stuff that updates the UI
                    pcolorPixelsText = (TextView) findViewById(R.id.peanutPixels);
                    pcolorPixelsText.setText("" +  backgroundPixels);

                }
        });

        if(backgroundPixels < 14900 && prevColor==0){
            prevColor = 0;
            currentColor = 1;
        }

        if(backgroundPixels > 14900 && prevColor == 1){
            prevColor = 0;
            currentColor = 0;
        }

        //If backgroundPixel count is less than 14900, then an object is present so continue processing
        if(backgroundPixels < 14900) {

            //***********************1. Remove Shadows***************************************
            List<Mat> channel = new ArrayList<>(3);
            Core.split(hsvImg, channel);
            channel.set(2, new Mat(hsvImg.rows(), hsvImg.cols(), CV_8UC1, new Scalar(200)));//Set V
            //Merge channels
            Core.merge(channel, hsvImg);
            Mat rgbImg = new Mat();
            Imgproc.cvtColor(hsvImg, rgbImg, Imgproc.COLOR_HSV2RGB);


           // Mat rgbImg = processor.removeShadows(hsvImg);

            //*****************************2. Convert to gray and normalize**********************
            gray = new Mat(rgbImg.rows(), rgbImg.cols(), CV_8UC1);
            Imgproc.cvtColor(rgbImg, gray, Imgproc.COLOR_RGB2GRAY, 1);
            Core.normalize(gray, gray, 0, 255, Core.NORM_MINMAX, CV_8UC1);

            //gray = processor.grayScale(rgbImg);

            //*****************************3. Bilateral filter, threshold and Canny ****************************
            filter = new Mat();
            Imgproc.bilateralFilter(gray, filter, 9, 9 * 2, 9 / 2);
            edges = new Mat();
            Mat img = new Mat();
            double otsu_thresh_val = Imgproc.threshold(filter, img, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
            double high_thresh_val = otsu_thresh_val;
            double lower_thresh_val = otsu_thresh_val * 0.5;

            Imgproc.Canny(filter, edges, lower_thresh_val, high_thresh_val, 3, false);  //before 50, 225

         // edges = processor.filter(gray);
            //*******************************Find contours and find the biggest contour index****************************************************
            Mat src2 = new Mat();
            src.copyTo(src2);
            List<MatOfPoint> contours2 = new ArrayList<>(); //will contain all contours found in the image
            Mat hierarchy2 = new Mat();
            double contour_area2;
            Imgproc.findContours(edges, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE); //CV_RETR_CCOMP RETR_EXTERNAL
            Mat maskMat = new Mat(src.rows(), src.cols(), CV_8UC1, new Scalar(0, 0, 0));

            //int count2 = 0;
            int maxAreaContourId = -1;
            double maxArea = 0;
            //Check for the index of the largest contour
            for (int i = 0; i < contours2.size(); i++) {
                contour_area2 = Imgproc.contourArea(contours2.get(i));
                if (contour_area2 > 20) {
                   // count2++;
                    if (contour_area2 > maxArea) {
                        maxArea = contour_area2;
                        maxAreaContourId = i;
                    }
                }
            }


            maxAreaContourId = processor.findBiggestContour(contours2, maxArea);

//************************CLOSE CONTOURS USING CONVEX HULL***********************************************
            if (maxAreaContourId >= 0) {
                convex = new Mat(src.rows(), src.cols(), CV_8UC1, new Scalar(0, 0, 0));
                MatOfPoint mopIn = contours2.get(maxAreaContourId);
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(mopIn, hull, false);
                MatOfPoint mopOut = new MatOfPoint();
                mopOut.create((int) hull.size().height, 1, CV_32SC2);

                for (int i = 0; i < hull.size().height; i++) {
                    int index = (int) hull.get(i, 0)[0];
                    double[] point = new double[]{mopIn.get(index, 0)[0], mopIn.get(index, 0)[1]};
                    mopOut.put(i, 0, point);
                }

                //Draw polylines(closed contours), contours and fill them for calculating mean
                List<MatOfPoint> con = new ArrayList<>();
                con.add(mopOut);
                Imgproc.polylines(convex, con, true, new Scalar(255, 255, 255), 2);
                Imgproc.drawContours(convex, contours2, (maxAreaContourId), new Scalar(255, 255, 255), -1);
                Imgproc.fillPoly(convex, con, new Scalar(255, 255, 255));


                //DETECT RGB VALUE OF PEANUT WITH MEAN
                Scalar values = Core.mean(src, convex);
                hierarchy2.release();
                float[] hsv = new float[3];
                v1 = (int) values.val[0];
                 v2 = (int) values.val[1];
                v3 = (int) values.val[2];
                rgb = (v1 + " " + v2 + " " + v3);

              //  Mat rgbTohsv = new Mat(img.size(), CV_8UC3, new Scalar(values.val[0], values.val[1], values.val[2]));
              //  Imgproc.cvtColor(rgbTohsv, rgbTohsv, Imgproc.COLOR_RGB2HSV);

                Color.RGBToHSV(v1, v2, v3, hsv);
                h = hsv[0];
                s = hsv[1];
                v = hsv[2];

                hue = h/2;
                sat = s * 255;
                val = v* 255;


                //Uses the distance formula to categorize the object into a specific color
                if( prevColor == 0 && currentColor == 1) {
                    peanuts++; //increment the peanut detected
                    int minIndex = 0;
                    double min = Math.pow((Math.pow(hue - categories[0][0], 2) +
                            Math.pow(sat - categories[0][1], 2) +
                            Math.pow(val - categories[0][2], 2) *
                                    1.0), 0.5);
                    for(int  i = 0; i < categories.length; i++){
                        double distance = Math.pow((Math.pow(hue - categories[i][0], 2) +
                                Math.pow(sat - categories[i][1], 2) +
                                Math.pow(val - categories[i][2], 2) *
                                        1.0), 0.5);
                        if(distance < min){
                            minIndex = i;
                            min = distance;
                        }
                    }

                    if(val < 30){
                        colors[4]++;
                    }else {
                        if (minIndex == 0) {
                            colors[0]++;
                        } else if (minIndex == 1) {
                            colors[1]++;
                        } else if (minIndex == 2) {
                            colors[2]++;
                        } else if (minIndex == 3) {
                            colors[3]++;
                        } /*else if (minIndex == 4) {
                            colors[4]++;
                        }*/
                    }

                    prevColor = 1;
                }

                //set HSV values on UI
                runOnUiThread(new Runnable() {

                    @Override
                    public void run() {
                        h1 = (TextView) findViewById(R.id.hueText);
                        h1.setText(v1 + "\n " + v2 + "\n " + v3);
                    }
                });


                    //Draw contours on the source image
                    Scalar color1 = new Scalar(255, 0, 0, 255); // B G R values
                    Imgproc.polylines(src2, con, true, color1, 2);
                    Imgproc.drawContours(src2, contours2, (maxAreaContourId), color1, 2, 8, hierarchy2, 0, new Point());

                    //Mat to bitmap for the peanut color detected

                    masked = new Mat(img.size(), CV_8UC3, new Scalar(values.val[0], values.val[1], values.val[2]));
                    mask = Bitmap.createBitmap(maskMat.cols(), maskMat.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(masked, mask);

                    //Mat to bitmap for source image with contours
                    bmp = Bitmap.createBitmap(src2.cols(), src2.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(src2, bmp);

                    //Display count, categories and HSV values
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            // Stuff that updates the UI
                            pCountText = (TextView) findViewById(R.id.count);
                            pCountText.setText("Count: " + peanuts);

                            categoryText = (TextView) findViewById(R.id.categories);
                            categoryText.setText("\n Yellow: " + colors[0] + "\n Orange: " + colors[1] + "\n Brown: " + (colors[2] +colors[3]) + "\n Black: " + colors[4] );

                            hsvView = (TextView) findViewById(R.id.hsvLabel);
                            hsvView.setText("\n Hue: " + hue + "\n Saturation: " + sat + "\n Value: " + val );

                            iv = (ImageView) findViewById(R.id.img2);
                            iv.setImageBitmap(bmp);

                            iv = (ImageView) findViewById(R.id.peanut_color);
                            iv.setImageBitmap(mask);


                        }
                    });

            }
        }

            long endTime = System.nanoTime();
            System.out.println("Opencv time: " + ((endTime-startTime)/ 1_000_000_000.0));
    }

    /**
     * Draws a rectangle view on camera
     */
    void drawArea()
    {
        canvas = surfaceHolder.lockCanvas();
        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
        //int xcenter = (surfaceView.getLeft() + surfaceView.getRight() )/2;
        //int ycenter = (surfaceView.getTop() + surfaceView.getBottom())/2;
        int xcenter = surfaceView.getWidth()/2;
        int ycenter = surfaceView.getHeight()/2;
        int left = xcenter - 125/2;
        int  top = ycenter - 50/2;
        int right = xcenter + 125/2;
        int bottom = ycenter + 50/2;
        rec = new Rect(left,top,right,bottom);

        canvas.drawRect(rec, mpaint);
        surfaceHolder.unlockCanvasAndPost(canvas);
    }


    /**
     * When start button is clicked, the boolean variable is set to true to start image processing
     * @param view
     */
    public void startProcessing (View view)
    {// mHanlder.postDelayed(task, 1000); // start repeated task, 1st task start with 1s delay
        startProcessing = true;
    }

    /**
     * When stop button is clicked, the boolean variable is set to true to stop image processing
     * @param view
     */
    public void stopProcessing(View view){
        startProcessing = false;
    }

    //****************************OPENCV LIBRARY LOAD*****************************************
    static {
        if(!OpenCVLoader.initDebug()){
            Log.d(TAG, "OpenCV not loaded");
            Mat srcMat = new Mat();
        } else {
            Log.d(TAG, "OpenCV loaded");
        }
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");

                    next = (Button) findViewById(R.id.imgpage);
                    next.setOnClickListener(new OnClickListener() {
                        public void onClick(View v) {

                            if(startProcessing == false)
                            {
                                setContentView(R.layout.image);
                                TextView txt = (TextView)findViewById(R.id.colornames);
                                txt.setText("\n Yellow: " + colors[0] + "\n Orange: " + colors[1] + "\n Brown: " + (colors[2] +colors[3]) + "\n Black: " + colors[4] );
                            }
                        }
                    });
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };


    /**
     * FOR RUNNABLE. NOT USED
     */
    private Handler mHanlder = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case 1:
                    SimpleDateFormat format1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
                    Date currentDate = new Date();
                    String displayTime1 = format1.format(currentDate);

                    // txtShow.setText(displayTime1);
                    break;
                default:
                    break;
            }
            super.handleMessage(msg);
        }
    };

    /**
     * Not used
     * For runnable thread, remove callback
     * @param view
     */
    public void stopRunnable(View view){
        mHanlder.removeCallbacks(task);
    }

    /**
     * NOT USED
     * A separate thread for running image processing that runs every 25 milliseconds
     */
    private Runnable task = new Runnable() {
        @Override
        public void run() {

            try {
                initializeOpenCVDependencies(); // repeated task
            } catch (IOException e) {
                e.printStackTrace();
            }
            mHanlder.sendEmptyMessage(1);
            mHanlder.postDelayed(this, 1 * 25);// time interval
        }
    };


    /**
     * opens camera if textview is  loaded and also loads opencv
     */
    @Override
    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        startBackgroundThread();

        if (textureView.isAvailable())
        {
            try {
                openCamera();
            } catch (CameraAccessException e) {
                e.printStackTrace();
            }
        }
        else
        {
            textureView.setSurfaceTextureListener(surfaceTextureListener);
        }
    }

    TextureView.SurfaceTextureListener surfaceTextureListener = new TextureView.SurfaceTextureListener() {
        @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int i, int i1) {
            try {
                openCamera();

            } catch (CameraAccessException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int i, int i1) {

        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
            return false;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {

        }
    };


    /**
     *Opens camera and handles permissions
     * @throws CameraAccessException
     */
    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    private void openCamera() throws CameraAccessException {
        cameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        cameraId = cameraManager.getCameraIdList()[0];

        CameraCharacteristics cc = cameraManager.getCameraCharacteristics(cameraId);
        StreamConfigurationMap map = cc.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        imageDimensions = map.getOutputSizes(SurfaceTexture.class)[0];


        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermission here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }

        cameraManager.openCamera(cameraId, stateCallback, null);
        mImageReader = ImageReader.newInstance(640, 480,
                ImageFormat.YUV_420_888, /*maxImages*/2);
        mImageReader.setOnImageAvailableListener(
                mOnImageAvailableListener, backgroundHandler);
    }


    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
        @Override
        public void onOpened(@NonNull CameraDevice camera) {
            cameraDevice = camera;
            try {
                startCameraPreview();
            } catch (CameraAccessException e) {
                e.printStackTrace();
            }
        }

        @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
        @Override
        public void onDisconnected(@NonNull CameraDevice camera) {
            cameraDevice.close();
        }

        @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
        @Override
        public void onError(@NonNull CameraDevice camera, int i) {
            cameraDevice.close();
            cameraDevice = null;
        }
    };


    /**
     * Retreived frames from camera for real time processing
     * COnverts imagereader to mat for image processing.
     */
    @TargetApi(KITKAT)
    private final ImageReader.OnImageAvailableListener mOnImageAvailableListener
            = new ImageReader.OnImageAvailableListener() {

        @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
        @Override
        public void onImageAvailable(ImageReader reader) {
            try {
                image = reader.acquireLatestImage();
                if (image != null) {

                    if(startProcessing== true) {
                        //********************CONVERT IMAGE YUV FORMAT TO MAT
                        byte[] nv21;
                        ByteBuffer yBuffer = image.getPlanes()[0].getBuffer();
                        ByteBuffer uBuffer = image.getPlanes()[1].getBuffer();
                        ByteBuffer vBuffer = image.getPlanes()[2].getBuffer();

                        int ySize = yBuffer.remaining();
                        int uSize = uBuffer.remaining();
                        int vSize = vBuffer.remaining();

                        nv21 = new byte[ySize + uSize + vSize];

                        //U and V are swapped
                        yBuffer.get(nv21, 0, ySize);
                        vBuffer.get(nv21, ySize, vSize);
                        uBuffer.get(nv21, ySize + vSize, uSize);

                        mRGB = getYUV2Mat(nv21);

                        //CROP THE MAT TO RECTANGLE FIELD OF VIEW AND SCALE DOWN THE MAT TO IMPROVE PROCESSING TIME
                        //crop width = 80, crop height = 200
                        int xc = mRGB.width()/2;
                        int yc = mRGB.height()/2;
                        int left = xc - 80/2;
                        int top = yc - 200/2;
                        org.opencv.core.Rect rectCrop = new org.opencv.core.Rect(left, top, 80, 200);
                       croppedMat = mRGB.submat(rectCrop);
                      // Size scaleSize = new Size(100,150);
                      // Imgproc.resize(croppedMat, croppedMat, scaleSize , 0, 0, Imgproc.INTER_AREA);
                        drawArea(); //draw rectangle field of view on camera
                        initializeOpenCVDependencies(); //start processing
                    }

                    if(image != null) {
                        image.close();// don't forget to close
                    }

                }
            } catch (Exception e) {
                Log.w(TAG, e.getMessage());
            }finally{

            }
        }

    };

    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    public Mat getYUV2Mat(byte[] data) {
        Mat mYuv = new Mat(image.getHeight() + image.getHeight() / 2, image.getWidth(), CV_8UC1);
        mYuv.put(0, 0, data);
        Mat mRGB = new Mat();
        Imgproc.cvtColor(mYuv, mRGB, Imgproc.COLOR_YUV2RGB_NV21, 3);
        return mRGB;
    }


    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    private void startCameraPreview() throws CameraAccessException {
        SurfaceTexture texture = textureView.getSurfaceTexture();
        texture.setDefaultBufferSize(640, 480);

        Surface surface = new Surface(texture);
        captureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        surfaces = new ArrayList<>();
        surfaces.add(surface);
        captureRequestBuilder.addTarget(surface);

        //ADD IMAGE READER SURFACE TO THE CAPTURE BUILDER (NEW CODE)
        Surface mImageSurface = mImageReader.getSurface();
        surfaces.add(mImageSurface);
        captureRequestBuilder.addTarget(mImageSurface);

        //CHANGE FRAME SPEED TO BE IN THE RANGE 5-15 FOR BETTER PERFORMANCE
        CameraCharacteristics cameraCharacteristics = cameraManager.getCameraCharacteristics(cameraId);
       // Range<Integer> fps[] = cameraCharacteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES); //android's fps range
       // System.out.println("fps: " + fps[0] + " " + fps[1] + " " + fps[fps.length - 1]);
        Range<Integer>[] fps = new Range[5];
        fps[0] = Range.create(5,15);
        captureRequestBuilder.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps[0]);

        //Arrays.asList(surface) for first parameter
        cameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {
            @Override
            public void onConfigured(@NonNull CameraCaptureSession session) {
                if(cameraDevice == null)
                {
                    return;
                }
                cameraSession =session;

                try {
                    updatePreview();
                } catch (CameraAccessException e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onConfigureFailed(@NonNull CameraCaptureSession session) {
            }
        },null);
    }

    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    private void updatePreview() throws CameraAccessException {
        if(cameraDevice == null)
        {
            return;
        }

        //FOR INCREASING OR DECREASING BRIGHTNESS
      // captureRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_AUTO);
      // captureRequestBuilder.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, 12);
      // captureRequestBuilder.set(CaptureRequest.COLOR_CORRECTION_GAINS, new RggbChannelVector(86, 86, 86, 86));

        captureRequestBuilder.set(CaptureRequest.CONTROL_AE_LOCK, false);
        cameraSession.setRepeatingRequest(captureRequestBuilder.build(),null,backgroundHandler);
    }

    private void startBackgroundThread()
    {
        handlerThread = new HandlerThread("Camera Background");
        handlerThread.start();
        backgroundHandler = new Handler(handlerThread.getLooper());
    }

    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    @Override
    protected void onPause() {
        super.onPause();
        try {
            stopBackgroundThread();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    private void stopBackgroundThread() throws InterruptedException {
        handlerThread.quitSafely();
        handlerThread.join();
        backgroundHandler = null;
        handlerThread = null;
    }

    private final BroadcastReceiver mBroadcastReceiver1 = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            // When discovery finds a device
            if (action.equals(mBluetoothAdapter.ACTION_STATE_CHANGED)) {
                final int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, mBluetoothAdapter.ERROR);

                switch(state){
                    case BluetoothAdapter.STATE_OFF:
                        Log.d(TAG, "onReceive: STATE OFF");
                        break;
                    case BluetoothAdapter.STATE_TURNING_OFF:
                        Log.d(TAG, "mBroadcastReceiver1: STATE TURNING OFF");
                        break;
                    case BluetoothAdapter.STATE_ON:
                        Log.d(TAG, "mBroadcastReceiver1: STATE ON");
                        break;
                    case BluetoothAdapter.STATE_TURNING_ON:
                        Log.d(TAG, "mBroadcastReceiver1: STATE TURNING ON");
                        break;
                }
            }
        }
    };

    /**
     * Broadcast Receiver for changes made to bluetooth states such as:
     * 1) Discoverability mode on/off or expire.
     */
    private final BroadcastReceiver mBroadcastReceiver2 = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            if (action.equals(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED)) {

                int mode = intent.getIntExtra(BluetoothAdapter.EXTRA_SCAN_MODE, BluetoothAdapter.ERROR);

                switch (mode) {
                    //Device is in Discoverable Mode
                    case BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE:
                        Log.d(TAG, "mBroadcastReceiver2: Discoverability Enabled.");
                        break;
                    //Device not in discoverable mode
                    case BluetoothAdapter.SCAN_MODE_CONNECTABLE:
                        Log.d(TAG, "mBroadcastReceiver2: Discoverability Disabled. Able to receive connections.");
                        break;
                    case BluetoothAdapter.SCAN_MODE_NONE:
                        Log.d(TAG, "mBroadcastReceiver2: Discoverability Disabled. Not able to receive connections.");
                        break;
                    case BluetoothAdapter.STATE_CONNECTING:
                        Log.d(TAG, "mBroadcastReceiver2: Connecting....");
                        break;
                    case BluetoothAdapter.STATE_CONNECTED:
                        Log.d(TAG, "mBroadcastReceiver2: Connected.");
                        break;
                }
            }
        }
    };

    /**
     * Broadcast Receiver for listing devices that are not yet paired
     * -Executed by btnDiscover() method.
     */
    private BroadcastReceiver mBroadcastReceiver3 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            Log.d(TAG, "onReceive: ACTION FOUND.");

            if (action.equals(BluetoothDevice.ACTION_FOUND)){
                BluetoothDevice device = intent.getParcelableExtra (BluetoothDevice.EXTRA_DEVICE);
                mBTDevices.add(device);
                Log.d(TAG, "onReceive: " + device.getName() + ": " + device.getAddress());
                mDeviceListAdapter = new DeviceListAdapter(context, R.layout.device_adapter_view, mBTDevices);
                lvNewDevices.setAdapter(mDeviceListAdapter);
            }
        }
    };

    /**
     * Broadcast Receiver that detects bond state changes (Pairing status changes)
     */
    private final BroadcastReceiver mBroadcastReceiver4 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();

            if(action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED)){
                BluetoothDevice mDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                //3 cases:
                //case1: bonded already
                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDED){
                    Log.d(TAG, "BroadcastReceiver: BOND_BONDED.");
                    //inside BroadcastReceiver4
                    mBTDevice = mDevice;
                }
                //case2: creating a bone
                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDING) {
                    Log.d(TAG, "BroadcastReceiver: BOND_BONDING.");
                }
                //case3: breaking a bond
                if (mDevice.getBondState() == BluetoothDevice.BOND_NONE) {
                    Log.d(TAG, "BroadcastReceiver: BOND_NONE.");
                }
            }
        }
    };

    /**
     * Unregistering bluetooth
     */
    @Override
    protected void onDestroy() {
        Log.d(TAG, "onDestroy: called.");
        super.onDestroy();
        unregisterReceiver(mBroadcastReceiver1);
        unregisterReceiver(mBroadcastReceiver2);
        unregisterReceiver(mBroadcastReceiver3);
        unregisterReceiver(mBroadcastReceiver4);
        //mBluetoothAdapter.cancelDiscovery();
    }

    BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String text = intent.getStringExtra("theMessage");
            messages.append(text);

        }

    };
    //create method for starting connection
//***remember the conncction will fail and app will crash if you haven't paired first
    public void startConnection(){
        startBTConnection(mBTDevice,MY_UUID_INSECURE);
    }

    /**
     * starting chat service method
     */
    public void startBTConnection(BluetoothDevice device, UUID uuid){
        Log.d(TAG, "startBTConnection: Initializing RFCOM Bluetooth Connection.");
        mBluetoothConnection.startClient(device,uuid);
    }

    public void enableDisableBT(){
        if(mBluetoothAdapter == null){
            Log.d(TAG, "enableDisableBT: Does not have BT capabilities.");
        }
        if(!mBluetoothAdapter.isEnabled()){
            Log.d(TAG, "enableDisableBT: enabling BT.");
            Intent enableBTIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivity(enableBTIntent);

            IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
            registerReceiver(mBroadcastReceiver1, BTIntent);
        }
        if(mBluetoothAdapter.isEnabled()){
            Log.d(TAG, "enableDisableBT: disabling BT.");
            mBluetoothAdapter.disable();

            IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
            registerReceiver(mBroadcastReceiver1, BTIntent);
        }

    }


    public void btnEnableDisable_Discoverable(View view) {
        Log.d(TAG, "btnEnableDisable_Discoverable: Making device discoverable for 300 seconds.");

        Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
        startActivity(discoverableIntent);

        IntentFilter intentFilter = new IntentFilter(mBluetoothAdapter.ACTION_SCAN_MODE_CHANGED);
        registerReceiver(mBroadcastReceiver2,intentFilter);

    }

    @RequiresApi(api = Build.VERSION_CODES.M)
    public void btnDiscover(View view) {
        Log.d(TAG, "btnDiscover: Looking for unpaired devices.");

        if(mBluetoothAdapter.isDiscovering()){
            mBluetoothAdapter.cancelDiscovery();
            Log.d(TAG, "btnDiscover: Canceling discovery.");

            //check BT permissions in manifest
            checkBTPermissions();

            mBluetoothAdapter.startDiscovery();
            IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND);
            registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
        }
        if(!mBluetoothAdapter.isDiscovering()){

            //check BT permissions in manifest
            checkBTPermissions();
            mBluetoothAdapter.startDiscovery();
            IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND);
            registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
        }
    }

    /**
     * This method is required for all devices running API23+
     * Android must programmatically check the permissions for bluetooth. Putting the proper permissions
     * in the manifest is not enough.
     * NOTE: This will only execute on versions > LOLLIPOP because it is not needed otherwise.
     */
    @RequiresApi(api = Build.VERSION_CODES.M)
    private void checkBTPermissions() {
        if(Build.VERSION.SDK_INT > Build.VERSION_CODES.LOLLIPOP){
            int permissionCheck = this.checkSelfPermission("Manifest.permission.ACCESS_FINE_LOCATION");
            permissionCheck += this.checkSelfPermission("Manifest.permission.ACCESS_COARSE_LOCATION");
            if (permissionCheck != 0) {

                this.requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION}, 1001); //Any number
            }
        }else{
            Log.d(TAG, "checkBTPermissions: No need to check permissions. SDK version < LOLLIPOP.");
        }
    }

    @Override
    public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
        //first cancel discovery because its very memory intensive.
        mBluetoothAdapter.cancelDiscovery();

        Log.d(TAG, "onItemClick: You Clicked on a device.");
        String deviceName = mBTDevices.get(i).getName();
        String deviceAddress = mBTDevices.get(i).getAddress();

        Log.d(TAG, "onItemClick: deviceName = " + deviceName);
        Log.d(TAG, "onItemClick: deviceAddress = " + deviceAddress);

        //create the bond.
        //NOTE: Requires API 17+? I think this is JellyBean
        if(Build.VERSION.SDK_INT > Build.VERSION_CODES.JELLY_BEAN_MR2){
            Log.d(TAG, "Trying to pair with " + deviceName);
            mBTDevices.get(i).createBond();

            mBTDevice = mBTDevices.get(i);
            mBluetoothConnection = new BluetoothConnectionService(MainActivity.this);
        }
    }
}



//android hsv- hue is 0-360, saturation is 0-1, value is 0 to 1
//opencv hsv hue is 0-180, saturation is 0-255, value is 0-255
