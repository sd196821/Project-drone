#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <libusb-1.0/libusb.h>
#include "libuvc/libuvc.h"
#include <libuvc/libuvc_config.h>
#include <libuvc/libuvc_internal.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

void cb(uvc_frame_t *frame, void *ptr)
{
  uvc_frame_t *bgr=NULL;
  uvc_error_t ret;
  IplImage* cvImg=NULL;
  cv::Mat matImg;

  //printf("callback! length = %u, ptr = %d\n", (unsigned int)frame->data_bytes, (int)ptr);
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if(!bgr){
    printf("unable to allocate bgr frame!\n");
    return;
  }
  ret = uvc_any2bgr(frame, bgr);
  if(ret){
    uvc_perror(ret, "uvc_any2bgr");
    return;
  }
  //cv::Mat Img(bgr->width, bgr->height);
  //cvImg.rows=bgr->height;
  //cvImg.cols=bgr->width;
  cvImg = cvCreateImageHeader(
                  cvSize(bgr->width, bgr->height),
                  IPL_DEPTH_8U,
              3);

  
  //cvImg=imread(bgr);
  cvSetData(cvImg, bgr->data, bgr->width * 3);
  //cvImg.data=(uchar*)bgr->data;
  matImg = cv::cvarrToMat(cvImg);
  cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Test1");
  cv::imshow("Test1", matImg);
  cvShowImage("Test",cvImg);
  cvWaitKey(10);
  //waitKey(10)

  cvReleaseImageHeader(&cvImg);

  uvc_free_frame(bgr);
}

void smd_cmd(uvc_device_handle_t *devh)
{
  unsigned char buf[2];
  int res=0;

  libusb_device_handle *dev_handle=NULL;

  buf[0] = 0x76;
  buf[1] = 0xc3;
  dev_handle = devh->usb_devh;
  res = libusb_claim_interface(devh->usb_devh, 0);
  if(res != LIBUSB_SUCCESS) printf("claim 0 is failed!\n");
  res = libusb_control_transfer(devh->usb_devh,
                0x21,
                0x01,
                0x0800,
                0x0600,
                buf,
                sizeof(buf),
                0);
  printf("res = %d\n",res);
  if(res == sizeof(buf))printf("Control transfer Success!\n");
  else printf("Control transfer Failed!\n");

  buf[0] = 0x04;
  buf[1] = 0x00;
  res = libusb_control_transfer(devh->usb_devh,
                0x21,
                0x01,
                0xa00,
                0x600,
                buf,
                sizeof(buf),
                0);
  if(res == sizeof(buf))printf("Control transfer Success!\n");
  else printf("Control transfer Failed!\n");
}

int main()
{
  //char *device = "/dev/video0";
  //struct uvc_device *dev;
  uvc_context_t *ctx=NULL;
  uvc_device_t *dev=NULL;
  uvc_device_handle_t *devh=NULL;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  uvc_error_t resAEMODE;
  uvc_error_t resEXP;


  unsigned int i,j;

  res = uvc_init(&ctx, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }
  puts("UVC initialized");

  res = uvc_find_device(
			ctx, &dev,
            0x18e3, 0x5031, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");
    
    
    res = uvc_open(dev, &devh);

    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      puts("Device opened");
      uvc_print_diag(devh, stderr);
      
      /* Try to negotiate a 640x480 30 fps YUYV stream profile */
      res = uvc_get_stream_ctrl_format_size(
					    devh, &ctrl, /* result stored in ctrl */
					    UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
					    640, 480, 30 /* width, height, fps */
					    );
      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);
      
      if(res < 0){
	uvc_perror(res, "get_mode");
      }else{
    uint8_t flags=0;//uint8_t
    //void* 12345;
    int aaa=12345;
    res = uvc_start_streaming(devh, &ctrl, cb,&aaa,flags);
	smd_cmd(devh);
	if(res < 0){
	  uvc_perror(res, "start_streaming");
	}else{
	  puts("Streaming for 10 seconds...");
	  resAEMODE = uvc_set_ae_mode(devh, 1);
	  uvc_perror(resAEMODE, "set_ae_mode");

	  while(1){
	    scanf("%d",&i);
	  }
	  uvc_stop_streaming(devh);
	  puts("Done streaming.");
	  
	}
	 
	uvc_close(devh);
	puts("Device closed");
      }
      uvc_unref_device(dev);
    }
  }
  uvc_exit(ctx);
  puts("UVC exited");
  
  return 0;
}
  


