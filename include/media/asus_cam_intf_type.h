//--------------------------------------------------------------------
//                     ASUSTek Computer Inc.
//         Copyright (c) 2013 ASUSTek Computer inc, Taipei.
//
//			7002 iCatch ISP Device Device
//--------------------------------------------------------------------
//File: asus_cam_intf_type.h
//Revision History:
//[2013.06.06]	Stimber_Hsueh created.


#ifndef __ASUS_CAM_INTF_TYPE_H__
#define __ASUS_CAM_INTF_TYPE_H__

#define FRAME_PERIOD    10		//ASUS_BSP LiJen "[A86][Camera][NA][Others]Camera mini porting"

typedef enum {
    CAM_INTF_PARM_HAL_VERSION,
    /* common between HAL1 and HAL3 */
    CAM_INTF_PARM_ANTIBANDING,
    CAM_INTF_PARM_EXPOSURE_COMPENSATION,
    CAM_INTF_PARM_AEC_LOCK,
    CAM_INTF_PARM_FPS_RANGE,
    CAM_INTF_PARM_AWB_LOCK,
    CAM_INTF_PARM_WHITE_BALANCE,
    CAM_INTF_PARM_EFFECT,
    CAM_INTF_PARM_BESTSHOT_MODE,
    CAM_INTF_PARM_DIS_ENABLE,
    CAM_INTF_PARM_LED_MODE,
    CAM_INTF_META_HISTOGRAM, /* 10 */
    CAM_INTF_META_FACE_DETECTION,
    CAM_INTF_META_AUTOFOCUS_DATA,

    /* specific to HAl1 */
    CAM_INTF_PARM_QUERY_FLASH4SNAP,
    CAM_INTF_PARM_EXPOSURE,
    CAM_INTF_PARM_SHARPNESS,
    CAM_INTF_PARM_CONTRAST,
    CAM_INTF_PARM_SATURATION,
    CAM_INTF_PARM_BRIGHTNESS,
    CAM_INTF_PARM_ISO,
    CAM_INTF_PARM_EXPOSURE_TIME,
    CAM_INTF_PARM_ZOOM, /* 20 */
    CAM_INTF_PARM_ROLLOFF,
    CAM_INTF_PARM_MODE,             /* camera mode */
    CAM_INTF_PARM_AEC_ALGO_TYPE,    /* auto exposure algorithm */
    CAM_INTF_PARM_FOCUS_ALGO_TYPE,  /* focus algorithm */
    CAM_INTF_PARM_AEC_ROI,
    CAM_INTF_PARM_AF_ROI,
    CAM_INTF_PARM_FOCUS_MODE,
    CAM_INTF_PARM_MANUAL_FOCUS_POS,
    CAM_INTF_PARM_SCE_FACTOR,
    CAM_INTF_PARM_FD,
    CAM_INTF_PARM_MCE, /* 30 */
    CAM_INTF_PARM_HFR,
    CAM_INTF_PARM_REDEYE_REDUCTION,
    CAM_INTF_PARM_WAVELET_DENOISE,
    CAM_INTF_PARM_HISTOGRAM,
    CAM_INTF_PARM_ASD_ENABLE,
    CAM_INTF_PARM_RECORDING_HINT,
    CAM_INTF_PARM_HDR,
    CAM_INTF_PARM_MAX_DIMENSION,
    CAM_INTF_PARM_RAW_DIMENSION,
    CAM_INTF_PARM_FRAMESKIP,
    CAM_INTF_PARM_ZSL_MODE,  /* indicating if it's running in ZSL mode */
    CAM_INTF_PARM_HDR_NEED_1X, /* if HDR needs 1x output */ /* 40 */
    CAM_INTF_PARM_LOCK_CAF,
    CAM_INTF_PARM_VIDEO_HDR,
    CAM_INTF_PARM_SENSOR_HDR,
    CAM_INTF_PARM_ROTATION,
    CAM_INTF_PARM_SCALE,
    CAM_INTF_PARM_VT, /* indicating if it's a Video Call Apllication */
    CAM_INTF_META_CROP_DATA,
    CAM_INTF_META_PREP_SNAPSHOT_DONE,
    CAM_INTF_META_GOOD_FRAME_IDX_RANGE,
    CAM_INTF_PARM_GET_CHROMATIX,
    CAM_INTF_PARM_SET_RELOAD_CHROMATIX,
    CAM_INTF_PARM_SET_AUTOFOCUSTUNING,
    CAM_INTF_PARM_GET_AFTUNE,
    CAM_INTF_PARM_SET_RELOAD_AFTUNE,
    CAM_INTF_PARM_SET_VFE_COMMAND,
    CAM_INTF_PARM_SET_PP_COMMAND,
    CAM_INTF_PARM_TINTLESS,
    CAM_INTF_PARM_CDS_MODE,
    CAM_INTF_PARM_WB_MANUAL, //J66
    CAM_INTF_PARM_LONGSHOT_ENABLE,

    /* stream based parameters */
    CAM_INTF_PARM_DO_REPROCESS,
    CAM_INTF_PARM_SET_BUNDLE,
    CAM_INTF_PARM_STREAM_FLIP,
    CAM_INTF_PARM_GET_OUTPUT_CROP,

    CAM_INTF_PARM_AF_MOBICAT_CMD,
    CAM_INTF_PARM_EZTUNE_CMD,
    CAM_INTF_PARM_INT_EVT,

    /* specific to HAL3 */
    /* Whether the metadata maps to a valid frame number */
    CAM_INTF_META_FRAME_NUMBER_VALID,
    /* COLOR CORRECTION.*/
    CAM_INTF_META_COLOR_CORRECT_MODE,
    /* A transform matrix to chromatically adapt pixels in the CIE XYZ (1931)
     * color space from the scene illuminant to the sRGB-standard D65-illuminant. */
    CAM_INTF_META_COLOR_CORRECT_TRANSFORM, /* 50 */
    /* CONTROL */
//    CAM_INTF_META_REQUEST_ID,
    /* A frame counter set by the framework. Must be maintained unchanged in
     * output frame. */
    CAM_INTF_META_FRAME_NUMBER,
    /* Whether AE is currently updating the sensor exposure and sensitivity
     * fields */
    CAM_INTF_META_AEC_MODE,
    /* List of areas to use for metering */
    CAM_INTF_META_AEC_ROI,
    /* Whether the HAL must trigger precapture metering.*/
    CAM_INTF_META_AEC_PRECAPTURE_TRIGGER,
    /* The ID sent with the latest CAMERA2_TRIGGER_PRECAPTURE_METERING call */
    CAM_INTF_META_AEC_PRECAPTURE_ID,
    /* Current state of AE algorithm */
    CAM_INTF_META_AEC_STATE,
    /* List of areas to use for focus estimation */
    CAM_INTF_META_AF_ROI,
    /* Whether the HAL must trigger autofocus. */
    CAM_INTF_META_AF_TRIGGER,
    /* Current state of AF algorithm */
    CAM_INTF_META_AF_STATE,
    /* The ID sent with the latest CAMERA2_TRIGGER_AUTOFOCUS call */
    CAM_INTF_META_AF_TRIGGER_ID,
    /* List of areas to use for illuminant estimation */
    CAM_INTF_META_AWB_REGIONS,
    /* Current state of AWB algorithm */
    CAM_INTF_META_AWB_STATE,
    /* Information to 3A routines about the purpose of this capture, to help
     * decide optimal 3A strategy */
    CAM_INTF_META_CAPTURE_INTENT,
    /* Overall mode of 3A control routines. We need to have this parameter
     * because not all android.control.* have an OFF option, for example,
     * AE_FPS_Range, aePrecaptureTrigger */
    CAM_INTF_META_MODE,
    /* DEMOSAIC */
    /* Controls the quality of the demosaicing processing */
    CAM_INTF_META_DEMOSAIC,
    /* EDGE */
    /* Operation mode for edge enhancement */
    CAM_INTF_META_EDGE,
    /* Control the amount of edge enhancement applied to the images.*/
    /* 1-10; 10 is maximum sharpening */
    CAM_INTF_META_SHARPNESS_STRENGTH,
    /* FLASH */
    /* Power for flash firing/torch, 10 is max power; 0 is no flash. Linear */
    CAM_INTF_META_FLASH_POWER,
    /* Firing time of flash relative to start of exposure, in nanoseconds*/
    CAM_INTF_META_FLASH_FIRING_TIME,
    /* Current state of the flash unit */
    CAM_INTF_META_FLASH_STATE,
    /* GEOMETRIC */
    /* Operating mode of geometric correction */
    CAM_INTF_META_GEOMETRIC_MODE,
    /* Control the amount of shading correction applied to the images */
    CAM_INTF_META_GEOMETRIC_STRENGTH,
    /* HOT PIXEL */
    /* Set operational mode for hot pixel correction */
    CAM_INTF_META_HOTPIXEL_MODE,
    /* LENS */
    /* Size of the lens aperture */
    CAM_INTF_META_LENS_APERTURE,
    /* State of lens neutral density filter(s) */
    CAM_INTF_META_LENS_FILTERDENSITY,
    /* Lens optical zoom setting */
    CAM_INTF_META_LENS_FOCAL_LENGTH,
    /* Distance to plane of sharpest focus, measured from frontmost surface
     * of the lens */
    CAM_INTF_META_LENS_FOCUS_DISTANCE,
    /* The range of scene distances that are in sharp focus (depth of field) */
    CAM_INTF_META_LENS_FOCUS_RANGE,
    /* Whether optical image stabilization is enabled. */
    CAM_INTF_META_LENS_OPT_STAB_MODE,
    /* Current lens status */
    CAM_INTF_META_LENS_STATE,
    /* NOISE REDUCTION */
    /* Mode of operation for the noise reduction algorithm */
    CAM_INTF_META_NOISE_REDUCTION_MODE,
   /* Control the amount of noise reduction applied to the images.
    * 1-10; 10 is max noise reduction */
    CAM_INTF_META_NOISE_REDUCTION_STRENGTH,
    /* SCALER */
    /* Top-left corner and width of the output region to select from the active
     * pixel array */
    CAM_INTF_META_SCALER_CROP_REGION,
    /* SENSOR */
    /* Duration each pixel is exposed to light, in nanoseconds */
    CAM_INTF_META_SENSOR_EXPOSURE_TIME,
    /* Duration from start of frame exposure to start of next frame exposure,
     * in nanoseconds */
    CAM_INTF_META_SENSOR_FRAME_DURATION,
    /* Gain applied to image data. Must be implemented through analog gain only
     * if set to values below 'maximum analog sensitivity'. */
    CAM_INTF_META_SENSOR_SENSITIVITY,
    /* Time at start of exposure of first row */
    CAM_INTF_META_SENSOR_TIMESTAMP,
    /* SHADING */
    /* Quality of lens shading correction applied to the image data */
    CAM_INTF_META_SHADING_MODE,
    /* Control the amount of shading correction applied to the images.
     * unitless: 1-10; 10 is full shading compensation */
    CAM_INTF_META_SHADING_STRENGTH,
    /* STATISTICS */
    /* State of the face detector unit */
    CAM_INTF_META_STATS_FACEDETECT_MODE,
    /* Operating mode for histogram generation */
    CAM_INTF_META_STATS_HISTOGRAM_MODE,
    /* Operating mode for sharpness map generation */
    CAM_INTF_META_STATS_SHARPNESS_MAP_MODE,
    /* A 3-channel sharpness map, based on the raw sensor data,
     * If only a monochrome sharpness map is supported, all channels
     * should have the same data
     */
    CAM_INTF_META_STATS_SHARPNESS_MAP,

    /* TONEMAP */
    /* Table mapping blue input values to output values */
    CAM_INTF_META_TONEMAP_CURVE_BLUE,
    /* Table mapping green input values to output values */
    CAM_INTF_META_TONEMAP_CURVE_GREEN,
    /* Table mapping red input values to output values */
    CAM_INTF_META_TONEMAP_CURVE_RED,
    /* Tone map mode */
    CAM_INTF_META_TONEMAP_MODE,
    CAM_INTF_META_FLASH_MODE,
    CAM_INTF_META_ASD_HDR_SCENE_DATA,
    CAM_INTF_META_PRIVATE_DATA,
    CAM_INTF_PARM_STATS_DEBUG_MASK,
    CAM_INTF_PARM_ISP_DEBUG_MASK,
    CAM_INTF_PARM_ALGO_OPTIMIZATIONS_MASK,
    CAM_INTF_PARM_SENSOR_DEBUG_MASK,
    /* Indicates streams ID of all the requested buffers */
    CAM_INTF_META_STREAM_ID,
    CAM_INTF_PARM_FOCUS_BRACKETING,
    CAM_INTF_PARM_MULTI_TOUCH_FOCUS_BRACKETING,
    CAM_INTF_PARM_FLASH_BRACKETING,
    CAM_INTF_PARM_GET_IMG_PROP,
    
    ASUS_CAM_INTF_PARM_SET_SENSOR_OP_MODE,   //ASUS_BSP Stimber "Implement all sensor modes"
    ASUS_CAM_INTF_PARM_SET_SENSOR_VAL,      //ASUS_BSP jim3_lin "Pass G sensor data to ISP"
    ASUS_CAM_INTF_PARM_TAE,                 // ASUS_BSP LiJen "Implement TAE mode for ISP"
    ASUS_CAM_INTF_PARM_3DNR,			    //ASUS_BSP Stimber "Implement HDR/3D-NR features"
    ASUS_CAM_INTF_PARM_HDR,			        //ASUS_BSP Stimber "Implement HDR/3D-NR features"
    ASUS_CAM_INTF_PARM_ULTRAPIXEL,          // ASUS_BSP jim3_lin "Implement DIT postprocess"
    ASUS_CAM_INTF_PARM_TRIGGER_EVENT,       //ASUS_BSP jim3_lin "Fix Scene/Effect/WB setting order"
    ASUS_CAM_INTF_PARM_AURA_VALUE,          // ASUS_BSP jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"
    ASUS_CAM_INTF_PARM_WDR,                 // ASUS_BSP jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"
    ASUS_CAM_INTF_PARM_GYRO,				//ASUS_BSP Stimber "Implement gyro detect mode"
    ASUS_CAM_INTF_PARM_EIS,                 //ASUS_BSP bill_chen "Implement image stabilization"
    ASUS_CAM_INTF_PARM_MAX_FPS,				//ASUS_BSP Stimber "Implement max frame rate mode"
    CAM_INTF_PARM_MAX
} cam_intf_parm_type_t;

struct kernel_mct_event_control_parm_t{
  cam_intf_parm_type_t type;
  void *parm_data;
};


//ASUS_BSP +++ LiJen "Modify for asus yuv sensor param types"
//Copy from \hardware\qcom\camera\qcamera2\stack\common\Cam_types.h
/* Auto focus mode */
typedef enum {
    CAM_FOCUS_MODE_AUTO,
    CAM_FOCUS_MODE_INFINITY,
    CAM_FOCUS_MODE_MACRO,
    CAM_FOCUS_MODE_FIXED,
    CAM_FOCUS_MODE_EDOF,
    CAM_FOCUS_MODE_CONTINOUS_VIDEO,
    CAM_FOCUS_MODE_CONTINOUS_PICTURE,
    CAM_FOCUS_MODE_MANUAL,
    CAM_FOCUS_MODE_MAX
} cam_focus_mode_type;
//ASUS_BSP --- LiJen "Modify for asus yuv sensor param types"

// ASUS_BSP +++ jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"
// Copy from /hardware/qcom/camera/qcamera2/stack/common/Cam_types.h
typedef enum {
    CAM_ISO_MODE_AUTO,
    CAM_ISO_MODE_DEBLUR,
    CAM_ISO_MODE_50, // jim add
    CAM_ISO_MODE_100,
    CAM_ISO_MODE_200,
    CAM_ISO_MODE_400,
    CAM_ISO_MODE_800,
    CAM_ISO_MODE_1600,
    CAM_ISO_MODE_3200,
    CAM_ISO_MODE_MAX
} cam_iso_mode_type;

typedef enum {
    CAM_WB_MODE_AUTO,
    CAM_WB_MODE_CUSTOM,
    CAM_WB_MODE_INCANDESCENT,
    CAM_WB_MODE_FLUORESCENT,
    CAM_WB_MODE_WARM_FLUORESCENT,
    CAM_WB_MODE_DAYLIGHT,
    CAM_WB_MODE_CLOUDY_DAYLIGHT,
    CAM_WB_MODE_TWILIGHT,
    CAM_WB_MODE_SHADE,
    CAM_WB_MODE_MANUAL, //J66
    CAM_WB_MODE_OFF,
    CAM_WB_MODE_MAX
} cam_wb_mode_type;
typedef enum {
    CAM_ANTIBANDING_MODE_OFF,
    CAM_ANTIBANDING_MODE_60HZ,
    CAM_ANTIBANDING_MODE_50HZ,
    CAM_ANTIBANDING_MODE_AUTO,
    CAM_ANTIBANDING_MODE_AUTO_50HZ,
    CAM_ANTIBANDING_MODE_AUTO_60HZ,
    CAM_ANTIBANDING_MODE_MAX,
} cam_antibanding_mode_type;

typedef enum {
    CAM_FLASH_MODE_OFF,
    CAM_FLASH_MODE_AUTO,
    CAM_FLASH_MODE_ON,
    CAM_FLASH_MODE_TORCH,
    CAM_FLASH_MODE_MAX
} cam_flash_mode_t;
// ASUS_BSP --- jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement Scene/Effect mode for ISP"
// Copy from /hardware/qcom/camera/qcamera2/stack/common/Cam_types.h
typedef enum {
    CAM_SCENE_MODE_OFF,
    CAM_SCENE_MODE_AUTO,
    CAM_SCENE_MODE_LANDSCAPE,
    CAM_SCENE_MODE_SNOW,
    CAM_SCENE_MODE_BEACH,
    CAM_SCENE_MODE_SUNSET,
    CAM_SCENE_MODE_NIGHT,
    CAM_SCENE_MODE_PORTRAIT,
    CAM_SCENE_MODE_BACKLIGHT,
    CAM_SCENE_MODE_SPORTS,
    CAM_SCENE_MODE_ANTISHAKE,
    CAM_SCENE_MODE_FLOWERS,
    CAM_SCENE_MODE_CANDLELIGHT,
    CAM_SCENE_MODE_FIREWORKS,
    CAM_SCENE_MODE_PARTY,
    CAM_SCENE_MODE_NIGHT_PORTRAIT,
    CAM_SCENE_MODE_THEATRE,
    CAM_SCENE_MODE_ACTION,
    CAM_SCENE_MODE_AR,
    /* DIT_ADD +++++ J66 */
    CAM_SCENE_MODE_HILIGHT,//ASUS_BSP Zhengwei_Cai "Add HiLight Best Shot interface"
    CAM_SCENE_MODE_HILIGHT_SNAPSHOT, /* 20 */	//add by sam for hi-light mode capture
    CAM_SCENE_MODE_NIGHT_SNAPSHOT,	//add by sam for night mode capture
    CAM_SCENE_MODE_HDR_SNAPSHOT,	//add by sam for hdr mode capture
    /* DIT_ADD ----- J66 */
    CAM_SCENE_MODE_FACE_PRIORITY,
    CAM_SCENE_MODE_BARCODE,
    CAM_SCENE_MODE_HDR,
    /* DIT_ADD +++++ J66*/
    CAM_SCENE_MODE_VIDEO,
    CAM_SCENE_MODE_VIDEO_EIS,
    /* DIT_ADD ----- J66*/
    CAM_SCENE_MODE_MAX
} cam_scene_mode_type;

typedef enum {
    CAM_EFFECT_MODE_OFF,
    CAM_EFFECT_MODE_MONO,
    CAM_EFFECT_MODE_NEGATIVE,
    CAM_EFFECT_MODE_SOLARIZE,
    CAM_EFFECT_MODE_SEPIA,
    CAM_EFFECT_MODE_POSTERIZE,
    CAM_EFFECT_MODE_WHITEBOARD,
    CAM_EFFECT_MODE_BLACKBOARD,
    CAM_EFFECT_MODE_AQUA,
    CAM_EFFECT_MODE_EMBOSS,
    CAM_EFFECT_MODE_SKETCH,
    CAM_EFFECT_MODE_NEON,
    CAM_EFFECT_MODE_MAX,
    // asus +++
    CAM_EFFECT_MODE_AURA,
    CAM_EFFECT_MODE_VINTAGE,
    CAM_EFFECT_MODE_VINTAGE2,
    CAM_EFFECT_MODE_LOMO,
    CAM_EFFECT_MODE_RED,
    CAM_EFFECT_MODE_BLUE,
    CAM_EFFECT_MODE_GREEN
    // asus ---
} cam_effect_mode_type;
// ASUS_BSP --- jim3_lin "Implement Scene/Effect mode for ISP"

//ASUS_BSP +++iJen "[A68][Camera][NA][Fix]implement ROI autofocus"
//Copy from \hardware\qcom\camera\qcamera2\stack\common\Cam_types.h
typedef struct  {
    int32_t left;
    int32_t top;
    int32_t width;
    int32_t height;
} cam_rect_t;

typedef struct  {
    cam_rect_t rect;
    int32_t weight; /* weight of the area, valid for focusing/metering areas */
} cam_area_t;
//ASUS_BSP --- LiJen "[A68][Camera][NA][Fix]implement ROI autofocus"

//ASUS_BSP +++ Stimber "Implement all sensor modes"
typedef enum {
    QCAMERA_OP_MODE_1,
	QCAMERA_OP_MODE_2,
	QCAMERA_OP_MODE_3,
	QCAMERA_OP_MODE_4,
	QCAMERA_OP_MODE_5,
	QCAMERA_OP_MODE_6,
	QCAMERA_OP_MODE_7,
	QCAMERA_OP_MODE_8,
	//QCAMERA_OP_MODE_9,
	QCAMERA_OP_MODE_10,
	QCAMERA_OP_MODE_11,
    QCAMERA_OP_MODE_MAX
} qcamera_op_mode_t;
//ASUS_BSP --- Stimber "Implement all sensor modes"

//ASUS_BSP LiJen +++ "[A86][Camera][NA][Others]Camera mini porting"
struct exif_cfg {
      uint16_t iso;
      uint16_t exp_time_num;    // Numerator
      uint16_t exp_time_denom;  // Denominator
      uint16_t flash_mode;
      uint32_t edge;
      unsigned char info_3a[24];
      uint16_t Yaverage;
      uint16_t scene_info;
};
//ASUS_BSP LiJen --- "[A86][Camera][NA][Others]Camera mini porting"

#endif /* __ASUS_CAM_INTF_TYPE_H__ */

