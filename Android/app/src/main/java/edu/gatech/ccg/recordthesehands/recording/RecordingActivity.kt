/**
 * RecordingActivity.kt
 * This file is part of Record These Hands, licensed under the MIT license.
 *
 * Copyright (c) 2021-23
 *   Georgia Institute of Technology
 *   Authors:
 *     Sahir Shahryar <contact@sahirshahryar.com>
 *     Matthew So <matthew.so@gatech.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package edu.gatech.ccg.recordthesehands.recording

import android.Manifest
import android.annotation.SuppressLint
import android.content.ContentValues
import android.content.Context
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.content.res.AssetFileDescriptor
import android.content.res.ColorStateList
import android.graphics.Bitmap
import android.graphics.ColorMatrix
import android.graphics.ColorMatrixColorFilter
import android.graphics.ImageFormat
import android.graphics.PorterDuff
import android.hardware.camera2.*
import android.media.ExifInterface
import android.media.ExifInterface.TAG_IMAGE_DESCRIPTION
import android.media.MediaCodec
import android.media.MediaPlayer
import android.media.MediaRecorder
import android.media.ThumbnailUtils
import android.os.*
import android.provider.MediaStore
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.*
import android.widget.Button
import android.widget.ImageView
import android.widget.TextView
import android.widget.Toast
import android.widget.VideoView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.lifecycle.lifecycleScope
import androidx.viewbinding.ViewBinding
import androidx.viewpager2.widget.ViewPager2
import com.google.android.material.floatingactionbutton.FloatingActionButton
import edu.gatech.ccg.recordthesehands.*
import edu.gatech.ccg.recordthesehands.Constants.APP_VERSION
import edu.gatech.ccg.recordthesehands.Constants.RECORDINGS_PER_WORD
import edu.gatech.ccg.recordthesehands.Constants.WORDS_PER_SESSION
import edu.gatech.ccg.recordthesehands.Constants.RESULT_CAMERA_DIED
import edu.gatech.ccg.recordthesehands.Constants.RESULT_NO_ERROR
import edu.gatech.ccg.recordthesehands.Constants.RESULT_RECORDING_DIED
import edu.gatech.ccg.recordthesehands.Constants.TABLET_SIZE_THRESHOLD_INCHES
import edu.gatech.ccg.recordthesehands.databinding.ActivityRecordBinding
import edu.gatech.ccg.recordthesehands.databinding.ActivityRecordTabletBinding
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import java.io.BufferedInputStream
import java.io.File
import java.io.FileInputStream
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.*
import java.util.concurrent.locks.ReentrantLock
import kotlin.collections.ArrayList
import kotlin.concurrent.withLock
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException
import kotlin.coroutines.suspendCoroutine
import kotlin.math.sqrt

/**
 * Contains the data for a clip within the greater recording.
 *
 * @param file       (File) The filename for the (overall) video recording.
 * @param videoStart (Date) The timestamp that the overall video recording started at.
 * @param signStart  (Date) The timestamp that the clip within the video started at.
 * @param signEnd    (Date) The timestamp that the clip within the video ended at.
 * @param isValid    (Boolean) true if this clip should be considered usable data, false otherwise.
 *                   We assume that if the user creates a new recording for a particular word, then
 *                   there was something wrong with their previous recording, and we then mark that
 *                   recording as invalid.
 */
data class ClipDetails(
  val file: File, val videoStart: Date, val signStart: Date,
  val signEnd: Date, var isValid: Boolean
) {

  /**
   * Creates a string representation for this recording. Used when sending confirmation emails.
   */
  override fun toString(): String {
    val sdf = SimpleDateFormat("yyyy_MM_dd_HH_mm_ss.SSS", Locale.US)
    val isValidPython = if (isValid) "True" else "False"
    return "(file=${file.absolutePath}, videoStart=${sdf.format(videoStart)}, " +
        "signStart=${sdf.format(signStart)}, signEnd=${sdf.format(signEnd)}, " +
        "isValid=$isValidPython)"
  }

  /**
   * Creates a string representation for this recording with an attached attempt number.
   */
  fun toString(attempt: Int = 1): String {
    val sdf = SimpleDateFormat("yyyy_MM_dd_HH_mm_ss.SSS", Locale.US)
    val isValidPython = if (isValid) "True" else "False"
    return "(file=${file.absolutePath}, videoStart=${sdf.format(videoStart)}, " +
        "signStart=${sdf.format(signStart)}, signEnd=${sdf.format(signEnd)}, " +
        "isValid=$isValidPython, attempt=$attempt)"
  }

}

/**
 * This class handles the recording of ASL into videos.
 *
 * @author  Matthew So <matthew.so@gatech.edu>, Sahir Shahryar <contact@sahirshahryar.com>
 * @since   October 4, 2021
 * @version 1.1.0
 */
class RecordingActivity : AppCompatActivity() {
  companion object {
    private val TAG = RecordingActivity::class.java.simpleName

    /**
     * Record video at 15 Mbps. At 1944x2592 @ 30 fps, this level of detail should be more
     * than high enough.
     */
    private const val RECORDER_VIDEO_BITRATE: Int = 15_000_000

    /**
     * Height, width, and frame rate of the video recording. Using a 4:3 aspect ratio allows us
     * to get the widest possible field of view on a Pixel 4a camera, which has a 4:3 sensor.
     * Any other aspect ratio would result in some degree of cropping.
     */
    private const val RECORDING_HEIGHT = 2592
    private const val RECORDING_WIDTH = 1944
    private const val RECORDING_FRAMERATE = 30

    private const val MAXIMUM_RESOLUTION = 6_000_000

  }


  private lateinit var chosenSize: Size

  // UI elements
  private var recorderPrepared = false

  /**
   * Big red button used to start/stop a clip. (Note that we are continuously recording;
   * the button only marks when the user started or stopped signing to the camera.)
   *
   * Note that this button can be either a FloatingActionButton or a Button, depending on
   * whether we are on a smartphone or a tablet, respectively.
   */
  lateinit var recordButton: View


  /**
   * The recording preview.
   */
  lateinit var cameraView: SurfaceView


  // UI state variables
  /**
   * Marks whether the user is using a tablet (diagonal screen size > 7.0 inches (~17.78 cm)).
   */
  private var isTablet = false

  /**
   * Marks whether or not the camera has been successfully initialized. This is used to prevent
   * parts of the code related to camera initialization from running multiple times.
   */
  private var cameraInitialized = false

  /**
   * Marks whether or not the camera is currently recording or not. We record continuously as soon
   * as the activity launches, so this value will be true in some instances that `isSigning` may
   * be false.
   */
  private var isRecording = false

  /**
   * A mutex lock for the recording button. The lock ensures that multiple quick taps of the
   * record button can't cause thread safety issues.
   */
  private val buttonLock = ReentrantLock()



  // Camera API variables
  /**
   * The thread for handling camera-related actions.
   */
  private lateinit var cameraThread: HandlerThread

  /**
   * The Handler object for accessing the camera. We primarily use this when initializing or
   * shutting down the camera.
   */
  private lateinit var cameraHandler: Handler

  /**
   * The buffer to which camera frames are projected. This is used by the MediaRecorder to
   * record the video
   */
  private lateinit var recordingSurface: Surface

  /**
   * The camera recorder instance, used to set up and control the recording settings.
   */
  private lateinit var recorder: MediaRecorder


  /**
   * A CameraCaptureSession object, which functions as a wrapper for handling / stopping the
   * recording.
   */
  private lateinit var session: CameraCaptureSession

  /**
   * Details of the camera being used to record the video.
   */
  private lateinit var camera: CameraDevice

  /**
   * The buffer to which the camera sends frames for the purposes of displaying a live preview
   * (rendered in the `cameraView`).
   */
  private var previewSurface: Surface? = null

  /**
   * The operating system's camera service. We can get camera information from this service.
   */
  private val cameraManager: CameraManager by lazy {
    val context = this.applicationContext
    context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
  }

  private val memoryFileAllocation = 20 * 1000 * 1000

  private val recordingDestination: File by lazy {
    Log.d(TAG, "Initializing memory file")
    File.createTempFile("input.mp4", "")
  }

  // Permissions
  /**
   * Marks whether the user has enabled the necessary permissions to record successfully. If
   * we don't check this, the app will crash instead of presenting an error.
   */
  private var permissions: Boolean = true

  /**
   * When the activity starts, this routine checks the CAMERA and WRITE_EXTERNAL_STORAGE
   * permissions. (We do not need the MICROPHONE permission as we are just recording silent
   * videos.)
   */
  val permission =
    registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { map ->
      map.entries.forEach { entry ->
        when (entry.key) {
          Manifest.permission.CAMERA ->
            permissions = permissions && entry.value
        }
      }
    }

  /**
   * A function to initialize a new thread for camera-related code to run on.
   */
  private fun generateCameraThread() = HandlerThread("CameraThread").apply { start() }

  /**
   * Generates a new [Surface] for storing recording data, which will promptly be assigned to
   * the [recordingSurface] field above.
   */
  private fun createRecordingSurface(recordingSize: Size): Surface {
    val surface = MediaCodec.createPersistentInputSurface()
    recorder = MediaRecorder(this)

    setRecordingParameters(recorder, surface, recordingSize).prepare()
    recorderPrepared = true

    return surface
  }

  /**
   * Prepares a [MediaRecorder] using the given surface.
   */
  private fun setRecordingParameters(rec: MediaRecorder, surface: Surface, recordingSize: Size) =
    rec.apply {
      // Set the video settings from our predefined constants.
      setVideoSource(MediaRecorder.VideoSource.SURFACE)
      setOutputFormat(MediaRecorder.OutputFormat.MPEG_4)
      setOutputFile(recordingDestination)
      setVideoEncodingBitRate(RECORDER_VIDEO_BITRATE)

      setVideoFrameRate(RECORDING_FRAMERATE)
      setVideoSize(recordingSize.width, recordingSize.height)

      setVideoEncoder(MediaRecorder.VideoEncoder.HEVC)
      setInputSurface(surface)

      /**
       * The orientation of 270 degrees (-90 degrees) was determined through
       * experimentation. For now, we do not need to support other
       * orientations than the default portrait orientation.
       *
       * The tablet orientation of 0 degrees is designed primarily to support the use of
       * a Pixel Tablet (2023) with its included stand (although any tablet with a stand
       * may suffice).
       */
      setOrientationHint(if (isTablet) 0 else 270)
    }

  private fun checkCameraPermission(): Boolean {
    /**
     * First, check camera permissions. If the user has not granted permission to use the
     * camera, give a prompt asking them to grant that permission in the Settings app, then
     * relaunch the app.
     */
    if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {

      val errorRoot = findViewById<ConstraintLayout>(R.id.main_root)
      val errorMessage = layoutInflater.inflate(
        R.layout.permission_error, errorRoot,
        false
      )
      errorRoot.addView(errorMessage)

      // We essentially "gray out" the record button using the ARGB code #FFFA9389
      // (light pink).
      recordButton.backgroundTintList = ColorStateList.valueOf(0xFFFA9389.toInt())

      // Since the user hasn't granted camera permissions, we need to stop here.
      return false
    }

    return true
  }

  private fun getFrontCamera(): String {
    for (id in cameraManager.cameraIdList) {
      val face = cameraManager.getCameraCharacteristics(id)
        .get(CameraCharacteristics.LENS_FACING)
      if (face == CameraSelector.LENS_FACING_FRONT) {
        return id
      }
    }

    throw IllegalStateException("No front camera available")
  }

  /**
   * This code initializes the camera-related portion of the code, adding listeners to enable
   * video recording as long as we hold down the Record button.
   */
  @SuppressLint("ClickableViewAccessibility")
  private fun initializeCamera() = lifecycleScope.launch(Dispatchers.Main) {
    Log.d(TAG, "Called initializeCamera()")
    if (!checkCameraPermission()) {
      Log.d(TAG, "Don't have permission to run the camera!")
      return@launch
    }

    /**
     * User has given permission to use the camera. First, find the front camera. If no
     * front-facing camera is available, crash. (This shouldn't fail on any modern
     * smartphone.)
     */
    val cameraId = getFrontCamera()

    /**
     * Open the front-facing camera.
     */
    camera = openCamera(cameraManager, cameraId, cameraHandler)

    /**
     * Send video feed to both [previewSurface] and [recordingSurface], then start the
     * recording.
     */
    val targets = listOf(previewSurface!!, recordingSurface)
    session = createCaptureSession(camera, targets, cameraHandler)
    Log.d(TAG, "Successfully created capture session")

    // Lock screen orientation
    this@RecordingActivity.requestedOrientation =
      ActivityInfo.SCREEN_ORIENTATION_LOCKED

    /**
     * Create a request to record at 30fps.
     */
    val cameraRequest = camera.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
      // Add the previewSurface buffer as a destination for the camera feed if it exists
      previewSurface?.let {
        addTarget(it)
      }

      // Add the recording buffer as a destination for the camera feed
      addTarget(recordingSurface)

      // Lock FPS at 30
      set(
        CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
        Range(RECORDING_FRAMERATE, RECORDING_FRAMERATE)
      )

      // Disable video stabilization. This ensures that we don't get a cropped frame
      // due to software stabilization.
      set(
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
        CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF
      )
      set(
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF
      )
    }.build()

    session.setRepeatingRequest(cameraRequest, null, cameraHandler)

    // Make sure the Record button is visible.
    recordButton.animate().apply {
      alpha(1.0f)
      duration = 250
    }.start()

    recordButton.visibility = View.VISIBLE
    recordButton.isClickable = true
    recordButton.isFocusable = true

    /**
     * Set a listener for when the user presses the record button.
     */
    if (!isTablet) {
      recordButton.setOnTouchListener { view, event ->
        return@setOnTouchListener smartphoneOnTouchListener(view, event)
      }
    } else {
      recordButton.setOnTouchListener { view, event ->
        return@setOnTouchListener tabletOnTouchListener(view, event)
      }
    }

  }

  private fun smartphoneOnTouchListener(view: View, event: MotionEvent): Boolean {
    /**
     * Do nothing if the record button is disabled.
     */

    when (event.action) {
      /**
       * User presses down the record button: mark the start of a recording.
       */
      MotionEvent.ACTION_DOWN -> lifecycleScope.launch(Dispatchers.IO) {
        Log.d(TAG, "Record button down")

        buttonLock.withLock {
          Log.d(TAG, "Recording starting")
          startRecording()
        }

        // Add a tint to the record button as feedback.
        runOnUiThread {
          val recFAB = recordButton as FloatingActionButton
          recFAB.backgroundTintList = ColorStateList.valueOf(0xFF7C0000.toInt())
          recFAB.setColorFilter(0x80ffffff.toInt(), PorterDuff.Mode.MULTIPLY)
        }
      }

      /**
       * User releases the record button: mark the end of the recording
       */
      MotionEvent.ACTION_UP -> lifecycleScope.launch(Dispatchers.IO) {
        Log.d(TAG, "Record button up")

        buttonLock.withLock {
          /**
           * Add this recording to the list of recordings for the currently-selected
           * word.
           */
          // TODO: Stop recording
          stopRecording()
          makePrediction()

          // Give the user some haptic feedback to confirm the recording is done.
          recordButton.performHapticFeedback(HapticFeedbackConstants.REJECT)

          runOnUiThread {
            recordButton.backgroundTintList = ColorStateList.valueOf(0xFFF80000.toInt())
            (recordButton as FloatingActionButton).clearColorFilter()
          }
        }
      }
    }

    return true
  }

  private fun tabletOnTouchListener(view: View, event: MotionEvent): Boolean {
    if (event.action == MotionEvent.ACTION_UP) {
      lifecycleScope.launch(Dispatchers.IO) {
        buttonLock.withLock {
          // User pressed "Start recording" button
          if (!isRecording) {
            Log.d(TAG, "Recording starting")
            startRecording()

            // Add a tint to the record button as feedback.
            runOnUiThread {
              recordButton.backgroundTintList = ColorStateList.valueOf(0xFF7C0000.toInt())
              (recordButton as Button).text = "STOP"
            }
          }

          // User pressed "Next phrase" button
          else {
            /**
             * Add this recording to the list of recordings for the currently-selected
             * word.
             */
            stopRecording()
            makePrediction()

            runOnUiThread {
              recordButton.backgroundTintList = ColorStateList.valueOf(0xFF2BA300.toInt())
              (recordButton as Button).text = "START"
            }

            isRecording = false
          }
        }
      }
    }

    return true
  }

  /**
   * Adds a callback to the camera view, which is used primarily to assign a value to
   * [previewSurface] once Android has finished creating the Surface for us. We need this
   * because we cannot initialize a Surface object directly but we still need to be able to
   * pass a Surface object around to the UI, which uses the contents of the Surface (buffer) to
   * render the camera preview.
   */
  private fun setupCameraCallback() {
    cameraView.holder.addCallback(object : SurfaceHolder.Callback {
      /**
       * Called when the OS has finished creating a surface for us.
       */
      override fun surfaceCreated(holder: SurfaceHolder) {
        Log.d(TAG, "Initializing surface!")
        previewSurface = holder.surface
        initializeCamera()
      }

      /**
       * Called if the surface had to be reassigned. In practical usage thus far, we have
       * not run into any issues here by not reassigning [previewSurface] when this callback
       * is triggered.
       */
      override fun surfaceChanged(holder: SurfaceHolder, format: Int, w: Int, h: Int) {
        Log.d(TAG, "New format, width, height: $format, $w, $h")
        Log.d(TAG, "Camera preview surface changed!")
      }

      /**
       * Called when the surface is destroyed. Typically this will occur when the activity
       * closes.
       */
      override fun surfaceDestroyed(holder: SurfaceHolder) {
        Log.d(TAG, "Camera preview surface destroyed!")
        previewSurface = null
      }
    })
  }


  /**
   * Opens up the requested Camera for a recording session.
   *
   * @suppress linter for "MissingPermission": acceptable here because this function is only
   * ever called from [initializeCamera], which exits before calling this function if camera
   * permission has been denied.
   */
  @SuppressLint("MissingPermission")
  private suspend fun openCamera(
    manager: CameraManager,
    cameraId: String,
    handler: Handler? = null
  ): CameraDevice = suspendCancellableCoroutine { caller ->
    manager.openCamera(
      cameraId, object : CameraDevice.StateCallback() {
        /**
         * Once the camera has been successfully opened, resume execution in the calling
         * function.
         */
        override fun onOpened(device: CameraDevice) {
          Log.d(TAG, "openCamera: New camera created with ID $cameraId")
          caller.resume(device)
        }

        /**
         * If the camera is disconnected, end the activity and return to the splash screen.
         */
        override fun onDisconnected(device: CameraDevice) {
          Log.e(TAG, "openCamera: Camera $cameraId has been disconnected")
          this@RecordingActivity.apply {
            setResult(RESULT_CAMERA_DIED)
            finish()
          }
        }

        /**
         * If there's an error while opening the camera, pass that exception to the
         * calling function.
         */
        override fun onError(device: CameraDevice, error: Int) {
          val msg = when (error) {
            ERROR_CAMERA_DEVICE -> "Fatal (device)"
            ERROR_CAMERA_DISABLED -> "Device policy"
            ERROR_CAMERA_IN_USE -> "Camera in use"
            ERROR_CAMERA_SERVICE -> "Fatal (service)"
            ERROR_MAX_CAMERAS_IN_USE -> "Maximum cameras in use"
            else -> "Unknown"
          }
          val exc = RuntimeException("Camera $cameraId error: ($error) $msg")
          Log.e(TAG, exc.message, exc)
          if (caller.isActive) {
            caller.resumeWithException(exc)
          }
        }
      },

      // Pass this code onto the camera handler thread
      handler
    )
  }

  /**
   * Create a CameraCaptureSession. This is required by the camera API
   */
  private suspend fun createCaptureSession(
    device: CameraDevice,
    targets: List<Surface>,
    handler: Handler? = null
  ): CameraCaptureSession = suspendCoroutine { cont ->
    /**
     * Set up the camera capture session with the success / failure handlers defined below
     */
    device.createCaptureSession(targets, object : CameraCaptureSession.StateCallback() {
      // Capture session successfully configured - resume execution
      override fun onConfigured(session: CameraCaptureSession) {
        cont.resume(session)
      }

      // Capture session config failed - throw exception
      override fun onConfigureFailed(session: CameraCaptureSession) {
        val exc = RuntimeException("Camera ${device.id} session configuration failed")
        Log.e(TAG, exc.message, exc)
        cont.resumeWithException(exc)
      }
    }, handler)
  }


  /**
   * Starts the camera recording once we have device and capture session information within
   * [initializeCamera].
   */
  private fun startRecording() {
    if (!recorderPrepared) {
      setRecordingParameters(recorder, recordingSurface, chosenSize).prepare()
      recorderPrepared = true
    }

    recorder.start()
    isRecording = true
  }

  /**
   * Stops the recording session once we release the Record button.
   */
  private fun stopRecording() {
    recorder.stop()
    isRecording = false
    recorderPrepared = false
  }


  var callCounter = 0
  private fun makePrediction() {
    // TODO: Load saved video, run through ML model
    val prediction = "sup$callCounter"
    callCounter += 1

    runOnUiThread {
        title = prediction
    }

  }

  /**
   * Handler code for when the activity restarts. Right now, we return to the splash screen if the
   * user exits mid-session, as the app is continuously recording throughout this activity's
   * lifespan.
   */
  override fun onRestart() {
    try {
      super.onRestart()
      // Shut down app when no longer recording
      // setResult(RESULT_RECORDING_DIED)
      // finish()
    } catch (exc: Throwable) {
      Log.e(TAG, "Error in RecordingActivity.onRestart()", exc)
    }
  }

  /**
   * Handles stopping the recording session.
   */
  override fun onStop() {
    try {
      // recorder.stop()
      session.stopRepeating()
      session.close()
      recorder.release()
      camera.close()
      cameraThread.quitSafely()
      recordingSurface.release()
      cameraHandler.removeCallbacksAndMessages(null)
      Log.d(TAG, "onStop: Stop and release all recording data")
      super.onStop()
    } catch (exc: Throwable) {
      Log.e(TAG, "Error in RecordingActivity.onStop()", exc)
    }
  }

  /**
   * Handle the activity being destroyed.
   */
  override fun onDestroy() {
    try {
      super.onDestroy()
    } catch (exc: Throwable) {
      Log.e(TAG, "Error in RecordingActivity.onDestroy()", exc)
    }
  }

  /**
   * Entry point for the RecordingActivity.
   */
  override fun onCreate(savedInstanceState: Bundle?) {
    super.onCreate(savedInstanceState)

    // Calculate the display size to determine whether to use mobile or tablet layout.
    val displayMetrics = resources.displayMetrics
    val heightInches = displayMetrics.heightPixels / displayMetrics.ydpi
    val widthInches = displayMetrics.widthPixels / displayMetrics.xdpi
    val diagonal = sqrt((heightInches * heightInches) + (widthInches * widthInches))
    Log.i(TAG, "Computed screen size: $diagonal inches")

    val binding: ViewBinding
    if (diagonal > TABLET_SIZE_THRESHOLD_INCHES) {
      isTablet = true
      binding = ActivityRecordTabletBinding.inflate(this.layoutInflater)
    } else {
      binding = ActivityRecordBinding.inflate(this.layoutInflater)
    }

    val view = binding.root
    setContentView(view)

    // Enable record button
    recordButton = findViewById(R.id.recordButton)
    recordButton.isHapticFeedbackEnabled = true
    recordButton.visibility = View.INVISIBLE

    // Set up the camera preview's size
    cameraView = findViewById(R.id.cameraPreview)
    cameraView.holder.setSizeFromLayout()

    val aspectRatioConstraint = findViewById<ConstraintLayout>(R.id.aspectRatioConstraint)
    val layoutParams = aspectRatioConstraint.layoutParams
    layoutParams.height = layoutParams.width * 4 / 3
    aspectRatioConstraint.layoutParams = layoutParams
  }

  /**
   * Handle activity resumption (typically from multitasking)
   */
  override fun onResume() {
    super.onResume()

    // Create camera thread
    cameraThread = generateCameraThread()
    cameraHandler = Handler(cameraThread.looper)

    val cameraId = getFrontCamera()
    val props = cameraManager.getCameraCharacteristics(cameraId)

    val sizes = props.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
    Log.i(TAG, sizes.toString())

    fun hasAspectRatio(heightRatio: Int, widthRatio: Int, dim: Size): Boolean {
      val target = heightRatio.toFloat() / widthRatio.toFloat()
      return ((dim.width.toFloat() / dim.height.toFloat()) - target < 0.01)
    }

    val heightRatio = if (isTablet) 4 else 3
    val widthRatio = if (isTablet) 3 else 4

    val largestAvailableSize = sizes?.getOutputSizes(ImageFormat.JPEG)?.filter {
      // Find a resolution smaller than the maximum pixel count (6 MP)
      // with an aspect ratio of either 4:3 or 3:4, depending on whether we are using
      // a tablet or not.
      it.width * it.height < MAXIMUM_RESOLUTION
          && (hasAspectRatio(heightRatio, widthRatio, it))
    }?.maxByOrNull { it.width * it.height }

    chosenSize = largestAvailableSize ?: Size(RECORDING_HEIGHT, RECORDING_WIDTH)

    Log.i(TAG, "Selected video resolution: ${chosenSize.width} x ${chosenSize.height}")
    recordingSurface = createRecordingSurface(chosenSize)

    if (!cameraInitialized) {
      setupCameraCallback()
      cameraInitialized = true
    }
  }

} // RecordingActivity