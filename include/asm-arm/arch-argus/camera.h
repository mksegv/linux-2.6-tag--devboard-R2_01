/*
 *  Copyright (C) 2003-2004 Axis AB and Anoto Group AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ASM_CAMERA_H
#define _ASM_CAMERA_H

/* camera _IOC_TYPE, bits 8 to 15 in ioctl cmd */

#define CAMIOCTYPE 42

/* supported ioctl _IOC_NR's */

// Action commands

#define CAMSTARTDMA      0x1  // Start a picture asynchronously
#define CAMABORTDMA      0x70 // Abort current DMA transfer
#define CAMSCCBSET       0x71 // Write to a sensor register over SCCB
#define CAMSETANALYZE    0x72 // Set analyze dimensions
#define CAMGETANALYZE    0x73 // Get result of the last analyze
#define CAMABORTALL      0x74 // Abort all requests
#define CAMSETCONVMATRIX 0x75 // Set the convolution matrix
#define CAMSETCONVSHIFT  0x76 // Set the convolution matrix shift factor
#define CAMENABLEGAMMA   0x77 // Enable/disable digital gamma
#define CAMSETGAMMA      0x78 // Set gamma segment nbr (23:16) start (15:8) slope (7:0)
#define CAMSETCOLCORR    0x79 // Set colcorr coefficient (20:19, 18:17) to value (15:0)
#define CAMSETCOMPR      0x7a // Set compr for color (19:18) to k (17:10) m (9:0)

/* set parameters */

#define SETDEFAULT        0x2 /* CCD/VIDEO/SS1M */
#define SETPARAMETERS     0x3 /* CCD/VIDEO      */
#define SETSIZE           0x4 /* CCD/VIDEO/SS1M */
#define SETCOMPRESSION    0x5 /* CCD/VIDEO/SS1M */
#define SETCOLORLEVEL     0x6 /* CCD/VIDEO      */
#define SETBRIGHTNESS     0x7 /* CCD            */
#define SETROTATION       0x8 /* CCD            */
#define SETTEXT           0x9 /* CCD/VIDEO/SS1M */
#define SETCLOCK          0xa /* CCD/VIDEO/SS1M */
#define SETDATE           0xb /* CCD/VIDEO/SS1M */
#define SETTIMEFORMAT     0xc /* CCD/VIDEO/SS1M */
#define SETDATEFORMAT     0xd /*     VIDEO      */
#define SETTEXTALIGNMENT  0xe /*     VIDEO      */
#define SETFPS            0xf /* CCD/VIDEO/SS1M */
#define SETCOMMENT       0xfe /* CCD/VIDEO      */
#define SETBADPIXELS     0xfd /* CCD            */
#define SETIMAGEFORMAT   0xfc /* CCD            */
#define SETCROPOFFSETX   0x60
#define SETCROPOFFSETY   0x61
#define SETSHARPNESS     0x62
#define SETCROPMODE      0x63

/* get parameters */

#define GETDRIVERTYPE    0x10 /* CCD/VIDEO/SS1M */
#define GETNBROFCAMERAS  0x11 /* CCD/VIDEO/SS1M */
#define GETPARAMETERS    0x12 /* CCD/VIDEO/SS1M */
#define GETBUFFERSIZE    0x13 /* CCD/VIDEO/SS1M */
#define GETIMAGEWIDTH    0x14 /* CCD            */
#define GETIMAGEHEIGHT   0x15 /* CCD            */
#define GETSENSORMODEL   0x16
#define GETDCYVALUES     0xa0 /* CCD      /SS1M */
#define GETDCYWIDTH      0xa1 /* CCD      /SS1M */
#define GETDCYHEIGHT     0xa2 /* CCD      /SS1M */
#define GETSIZE          0xa3 /* CCD/VIDEO      */
#define GETCOMPRESSION   0xa4 /* CCD/VIDEO      */

/* configure default parameters */

#define CONFIGUREDEFAULT  0x20 /* CCD/VIDEO/SS1M */
#define DEFSIZE           0x21 /* CCD/VIDEO/SS1M */
#define DEFCOMPRESSION    0x22 /* CCD/VIDEO/SS1M */
#define DEFCOLORLEVEL     0x23 /* CCD/VIDEO      */
#define DEFBRIGHTNESS     0x24 /* CCD            */
#define DEFROTATION       0x25 /* CCD            */
#define DEFWHITEBALANCE   0x26 /* CCD            */
#define DEFEXPOSURE       0x27 /* CCD            */
#define DEFAUTOEXPWINDOW  0x28 /* CCD            */
#define DEFTEXT           0x29 /* CCD/VIDEO/SS1M */
#define DEFCLOCK          0x2a /* CCD/VIDEO/SS1M */
#define DEFDATE           0x2b /* CCD/VIDEO/SS1M */
#define DEFTIMEFORMAT     0x2c /* CCD/VIDEO/SS1M */
#define DEFDATEFORMAT     0x2d /*     VIDEO      */
#define DEFTEXTALIGNMENT  0x2e /*     VIDEO      */
#define DEFFPS            0x2f /* CCD/VIDEO/SS1M */
#define DEFTEXTSTRING     0x30 /* CCD/VIDEO/SS1M */
#define DEFHEADERINFO     0x31 /* CCD/VIDEO/SS1M */
#define DEFWEXAR          0x32 /* CCD            */
#define DEFLINEDELAY      0x33 /* CCD            */
#define DEFDISABLEDVIDEO  0x34 /*     VIDEO      */
#define DEFVIDEOTYPE      0x35 /*     VIDEO      */
#define DEFMODULATION     0x36 /*     VIDEO      */
#define DEFXOFFSET        0x37 /*     VIDEO      */
#define DEFYOFFSET        0x38 /*     VIDEO      */
#define DEFYCMODE         0x39 /*     VIDEO      */
#define DEFVCRMODE        0x3a /*     VIDEO      */
#define DEFSTOREDCYVALUES 0x3b /* CCD      /SS1M */
#define DEFWCDS           0x3c /* CCD            */
#define DEFVGA            0x3d /*     VIDEO      */
#define DEFCOMMENT        0x3e /* CCD/VIDEO      */
#define DEFCOMMENTSIZE    0x3f /* CCD/VIDEO      */
#define DEFCOMMENTTEXT    0x50 /* CCD/VIDEO      */
#define DEFPRODUCTINFO    0x52 /* CCD/VIDEO      */


typedef enum {
        req_image = 0, 
        req_dummy = 1
} request_type;

typedef enum {
	format_jpeg = 0,    // full jpeg pipeline
	format_raw = 1,     // raw bayer pattern, no interpolation or jpeg
	format_rgb = 2      // interpolated and color corrected rgb, no jpeg
} image_format;

// The following values need to correspond to tables inside the driver.

typedef enum {
	res160x120 = 0,
	res320x240 = 1,
	res640x480 = 2,
	res800x600 = 3,
	res1024x768 = 4,
	res1280x1024 = 5,
	res1600x1200 = 6,
	res1280x720 = 7,
	res640x360 = 8,
	res1280x960 = 9,
	res1920x1536 = 10,
	res1920x1080 = 11,
	res720x288 = 12,
	res1440x288 = 13
} size_type;

// See if we can remove the following.

#define hugesize res640x480
#define fullsize res320x240
#define halfsize res160x120
  
typedef enum { 
        min       = 0,
        low       = 1,
        medium    = 2,
        high      = 3,
        very_high = 4,
        very_low  = 5
} compr_type;

typedef enum { 
        deg_0   = 0,
        deg_180 = 1,
        deg_90  = 2,
        deg_270 = 3
} rotation_type;

typedef enum { 
        auto_white    = 0,
        hold          = 1,
        fixed_outdoor = 2,
        fixed_indoor  = 3,
        fixed_fluor   = 4
} white_balance_type;

typedef enum { 
        auto_exp  = 0,
        fixed_exp = 1
} exposure_type;

typedef enum { 
        no_window = 0,
        center    = 1,
        top       = 2,
        lower     = 3,
        left      = 4,
        right     = 5,
        spot      = 6,
        cw        = 7
} exp_window_type;

typedef enum {
        h_24 = 0,
        h_12 = 1
} hour_type;

typedef enum {
        standard = 0,
        YYYY_MM_DD = 1,
        Www_Mmm_DD_YYYY = 2,
        Www_DD_MM_YYYY = 3,
        YYYY_MM_DD_hyphen = 4,
        Www_Mmm_DD_YYYY_space = 5,
        MM_DD_YYYY = 6,
        DD_MM_YYYY = 7
} date_type;

typedef enum {
	second = 0,
	hundreds = 1
} time_accuracy_type;

typedef enum {
        left_align = 0,
        center_align = 1,
        right_align = 2
} alignment_type;

typedef enum {
        text_top = 0,
        text_bottom = 1
} text_location_type;

typedef enum { 
        off = 0,
        on  = 1,
        no  = 0,
        yes = 1
} enable_type;

typedef enum {
        pal  = 0,
        ntsc = 1
} video_type;

typedef enum {
        pal_bghi_ntsc_m              = 0,
        ntsc_4_43_50hz_pal_4_43_60hz = 1,
        pal_n_ntsc_4_43_60hz         = 2,
        ntsc_n_pal_m                 = 3,
        secam_pal_4_43_60hz          = 4
} modulation_type;

typedef enum {
        cam0 = 0,
        cam1 = 1,
        cam2 = 2,
        cam3 = 3,
        quad = 32
} camera_type;

typedef enum {
        video_driver = 0,
        ccd_driver   = 1
} driver_type;

struct cam_param {
        request_type req_type;
        image_format format;
        size_type size;
        compr_type compression;
        rotation_type rotation;
        int color_level;
        int brightness;
        white_balance_type white_balance;
        exposure_type exposure;
        exp_window_type auto_exp_window;
        hour_type time_format;
        date_type date_format;
        time_accuracy_type time_accuracy;
        alignment_type text_alignment;
        text_location_type text_location;
        enable_type text;
        enable_type clock;
        enable_type date;
        enable_type fps;
        enable_type vga;
        enable_type comment;
	int cropmode;
	int cropoffsetx, cropoffsety;
};

struct video_param {
        enable_type disabled;
        modulation_type modulation;
        video_type video;
        enable_type signal;
        enable_type vcr;
        int xoffset;
        int yoffset;
};

/* The cam_request structure is used during the CAMSTARTDMA asynchronous
 * picture-taking ioctl call as an argument to specify a buffer which will get
 * the final picture.
 */

struct cam_request {
        char *buf;              // Pointer to the buffer to hold picture data
        unsigned int buflen;    // Length of the above buffer
        unsigned int size;      // Resulting length, 0 if the picture is not ready
        int done;
};

#define MAX_BAD_PIXELS  100
/* The bax_pixel_list struct specifies a list of pixels that should be
 * replaced by their neighbour */
struct bad_pixel_list
{
  int count;
  int x[MAX_BAD_PIXELS];
  int y[MAX_BAD_PIXELS];
};

// Used with the CAMGETANALYZE ioctl call to get a copy of the last
// analyze data present, along with the dimensions.

struct cam_analyze_request {
        unsigned int *buf;      // Pointer to analyze data buffer
        unsigned int buflen;    // Length of the above buffer
	unsigned int size;      // Resulting length in bytes
	unsigned int lasttime;  // Timestamp qualifier
        unsigned int blockwidth, blockheight; // Analyze block dimensions
	unsigned int numx, numy; // Number of blocks in both dimensions
};

// Normal color level corresponds to the manufacturer recommended
// color level for the used sensor, 0 to jpeg grayscale.
// Intermediate values are color jpeg but varying color saturation and
// values between 51 and 100 give an additional 20% boost of colors, although
// possibly at the expense of color space saturation and false colors.

#define NORMAL_COLOR_LEVEL 50
#define MAX_COLOR_LEVEL 100

#endif
