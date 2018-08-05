//
#include <jni.h>
#include <libavformat/avformat.h>
#include "log.h"
#include "inc/codec_api.h"
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>

static AVCodecContext  *pCodecCtx;
static AVCodec         *pCodec;
static AVFrame *pFrame;
static uint8_t *out_buffer;
static AVPacket packet;
static int ret, got_picture;

static AVCodecContext *c = NULL;
static int decodeFrame(AVCodecContext *dec_ctx, AVFrame *frame, AVPacket *pkt)
{
    int ret;

    ret = avcodec_send_packet(dec_ctx, pkt);
    if (ret < 0)
    {
        LOGE("Error sending a packet for decoding\n");
        return -1;
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_frame(dec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {//数据太少不能解出一帧，重新读取
            LOGE("No more data. %d", (ret));
            break;
        } else if (ret == AVERROR(EINVAL)) {//不支持数据，错误，退出
            LOGE("bbbbbbbb")
            return -1;
        } else if (ret < 0) {
            LOGE("Error during decoding\n");
            return -2;
        }

        LOGE("saving frame %3d, w=%d, h=%d", dec_ctx->frame_number, frame->width, frame->height);
    }
    return 0;
}
static int decode(AVCodecContext *codecCtx, AVFrame **f, AVPacket *pkt) {
    AVFrame* frame = (*f);
    int ret = 0;
    while (1)
    {
        while(1)
        {
            ret = avcodec_receive_frame(codecCtx, frame);
            if (ret == AVERROR(EAGAIN)) {//数据太少不能解出一帧，重新读取
                break;
            } else if (ret == AVERROR(EINVAL)) {//不支持数据，错误，退出
                LOGE("bbbbbbbb")
                return -1;
            }
            (*f) = frame;
            av_packet_unref(pkt);
            return 0;
        }
        if ((ret = avcodec_send_packet(codecCtx, pkt)) != 0)
        {
            LOGE("avcodec_send_packet error = %d", ret);
            return ret;
        }
    }
}
/*
static int x264_encode( AVFrame *frame, int width, int height )
{
    //int width, height;
    x264_param_t param;
    x264_picture_t pic;
    x264_picture_t pic_out;
    x264_t *h;
    int i_frame = 0;
    int i_frame_size;
    x264_nal_t *nal;
    int i_nal;

    FILE *file = fopen("/mnt/sdcard/scale.264", "wb");
    if (!file){
        LOGE("File open failed");
        goto fail;
    }

    // Get default params for preset/tuning
    if( x264_param_default_preset( &param, "ultrafast", "zerolatency" ) < 0 ){
        LOGE("x264_param_default_preset failed");
        goto fail;
    }

    // Configure non-default params
//    param.i_bitdepth = 8;
    param.i_csp = X264_CSP_I420;
    param.i_width  = width;
    param.i_height = height;
//    param.b_vfr_input = 0;
//    param.b_repeat_headers = 1;
//    param.b_annexb = 1;
//    param.rc.b_mb_tree = 0;
//    param.rc.i_rc_method = X264_RC_ABR;
//    param.i_level_idc = 30;
    param.b_intra_refresh = 0;
    param.i_log_level = X264_LOG_DEBUG;

    // Apply profile restrictions.
    if( x264_param_apply_profile( &param, "baseline" ) < 0 ) {
        LOGE(" x264_param_apply_profile failed");
        goto fail;
    }

    if( x264_picture_alloc( &pic, param.i_csp, param.i_width, param.i_height ) < 0 ) {
        LOGE("x264_picture_alloc failed");
        goto fail;
    }
#undef fail
#define fail fail2

    h = x264_encoder_open( &param );
    if( !h ) {
        LOGE(" x264_encoder_open failed");
        goto fail;
    }
#undef fail
#define fail fail3
    int luma_size = width * height;
    int i=0;
    // Encode frames
    for( ;; i_frame++ )
    {
        for(i = 0; i < 3; i++){
            pic.img.plane[i] = frame->data[i];
            pic.img.i_stride[i] = frame->linesize[i];
        }
        pic.i_pts = frame->pts;
        pic.i_type = X264_TYPE_AUTO;
        pic.img.i_csp = X264_CSP_I420;
        pic.img.i_plane = 3;

        i_frame_size = x264_encoder_encode( h, &nal, &i_nal, &pic, &pic_out );
        if( i_frame_size < 0 ) {
            LOGE("x264_encoder_encode failed");
            goto fail;
        }
        else if( i_frame_size )
        {
            LOGE(" hh");
            if( !fwrite( nal->p_payload, i_frame_size, 1, file ) ) {
                LOGE("fwrite failed 1");
                goto fail;
            }
        }
        break;
    }
    // Flush delayed frames
    while( x264_encoder_delayed_frames( h ) )
    {
        i_frame_size = x264_encoder_encode( h, &nal, &i_nal, NULL, &pic_out );
        if( i_frame_size < 0 ) {
            LOGE("x264_encoder_encode failed 2");
            goto fail;
        }
        else if( i_frame_size )
        {
            LOGE(" jj");
            if( !fwrite( nal->p_payload, i_frame_size, 1, file ) ) {
                LOGE("fwrite failed 2");
                goto fail;
            }
        }
    }
    x264_encoder_close( h );
    x264_picture_clean( &pic );
    fclose(file);
    return 0;

#undef fail
fail3:
    LOGE("fail3");
    x264_encoder_close( h );
fail2:
    LOGE("fail2");
    x264_picture_clean( &pic );
fail:
    LOGE("fail0");
    if (file) fclose(file);
    return -1;
}
*/

static void openh264_encode( AVFrame *frame, int width, int height )
{
    ISVCEncoder* encoder;
    int result = WelsCreateSVCEncoder(&encoder);
    if (result != 0)
    {
        LOGE("WelsCreateSVCEncoder failed");
        return;
    }

    //SEncParamBase encParam;
    SEncParamExt encParam;
    memset(&encParam, 0, sizeof(SEncParamExt));
    (*encoder)->GetDefaultParams(encoder, &encParam);
    encParam.iUsageType = CAMERA_VIDEO_REAL_TIME;
    encParam.fMaxFrameRate = 30;//fps
    encParam.iPicWidth = width;
    encParam.iPicHeight = height;
    encParam.iTargetBitrate = 10000;//bitrate;
    encParam.iRCMode = RC_QUALITY_MODE;
    encParam.iTemporalLayerNum = 1;
    encParam.iSpatialLayerNum = 1;
    encParam.bEnableDenoise = false;
    encParam.bEnableBackgroundDetection = true;
    encParam.bEnableAdaptiveQuant = false;
    encParam.bEnableFrameSkip = false;
    encParam.bEnableLongTermReference = false;
    encParam.uiIntraPeriod = 1;
    encParam.eSpsPpsIdStrategy = CONSTANT_ID;
    encParam.bPrefixNalAddingCtrl = false;
    encParam.sSpatialLayers[0].iVideoWidth = encParam.iPicWidth;
    encParam.sSpatialLayers[0].iVideoHeight = encParam.iPicHeight;
    encParam.sSpatialLayers[0].fFrameRate = encParam.fMaxFrameRate;
    encParam.sSpatialLayers[0].iSpatialBitrate = encParam.iTargetBitrate;
    encParam.sSpatialLayers[0].iMaxSpatialBitrate = encParam.iMaxBitrate;
//    encParam.sSpatialLayers[0].sSliceArgument.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
//    encParam.sSpatialLayers[0].sSliceArgument.uiSliceNum = 2;
    encParam.iLoopFilterDisableIdc      = 1;//!s->loopfilter;
    encParam.iEntropyCodingModeFlag     = 0;
    encParam.iMultipleThreadIdc         = 1;//avctx->thread_count;
    encParam.iMinQp = 20;
    encParam.iMaxQp = 30;
    encParam.bUseLoadBalancing = 0;
    encParam.bEnableLongTermReference = 0;

    (*encoder)->InitializeExt (encoder, &encParam);
    int videoFormat = videoFormatI420;
    (*encoder)->SetOption (encoder, ENCODER_OPTION_DATAFORMAT, &videoFormat);

    SFrameBSInfo info;
    memset (&info, 0, sizeof (SFrameBSInfo));
    SSourcePicture pic;
    memset (&pic, 0, sizeof (SSourcePicture));
    pic.iPicWidth = width;
    pic.iPicHeight = height;
    pic.iColorFormat = videoFormatI420;
    for (int i = 0; i < 3; i++)
    {
        pic.iStride[i] = frame->linesize[i];
        pic.pData[i]   = frame->data[i];
    }

    FILE *file = NULL;
    int rv = (*encoder)->EncodeFrame (encoder, &pic, &info);
    if (rv == cmResultSuccess && info.eFrameType != videoFrameTypeSkip)
    {
        file = fopen("/mnt/sdcard/encoded.264", "wb");
        if (!file){
            LOGE("File open failed");
            goto err_fail;
        }

        int size = 0;
        for (int i = 0; i < info.iLayerNum; ++i)
        {
            const SLayerBSInfo layerInfo = info.sLayerInfo[i];
            size_t layerSize = 0;
            for (int j = 0; j < layerInfo.iNalCount; ++j) {
                layerSize += layerInfo.pNalLengthInByte[j];
            }
            size += layerSize;
            LOGE("layerSize=%d", layerSize);
            if( !fwrite(layerInfo.pBsBuf, layerSize, 1, file) ) {
                LOGE("fwrite failed 1");
                goto err_fail;
            }
        }
        LOGE("Size=%d", size);
    } else {
        LOGE("EncodeFrame failure");
    }

err_fail:
    if (file) fclose(file);

    if (encoder) {
        (*encoder)->Uninitialize(encoder);
        WelsDestroySVCEncoder (encoder);
    }
}

JNIEXPORT jint JNICALL Java_com_hhbgk_example_androidyuvencoder_CodecWrapper_encode
        (JNIEnv *env, jclass jclazz, jbyteArray jin_packet, jint jw, jint jh, jint jdstw, jint jdsth)
{
    av_init_packet(&packet);
    unsigned char  *in_array = (unsigned char *) (*env)->GetByteArrayElements(env, jin_packet, 0);
    jsize  length = (jsize) (*env)->GetArrayLength(env, jin_packet);

    packet.size = (int) length;
    packet.data = in_array;
    int width = jw;
    int height = jh;

    int result = decode(pCodecCtx, &pFrame, &packet);
    LOGE("result=%d", result);
    if(result == 0)
    {
        int y_size=width * height;

        /*int f = open("/mnt/sdcard/video.yuv", O_CREAT|O_RDWR);
        if (f){
            write(f, buf, y_size*3/2);
            close(f);
        }*/
        struct SwsContext *resize;
        int dst_w = jdstw, dst_h = jdsth;
        LOGE("00 src w=%d, src h=%d, %d, %d", width, height, pFrame->width, pFrame->height);
        resize = sws_getContext(width, height, AV_PIX_FMT_YUV420P, dst_w, dst_h, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);
        AVFrame* pScaleAVFrame = av_frame_alloc();
        if (!pScaleAVFrame) goto err_out;
        int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, dst_w, dst_h, 16);
        uint8_t* out_buffer = (uint8_t *)av_malloc(num_bytes * sizeof(uint8_t));
        if (!out_buffer) goto err_out;
        ret = av_image_fill_arrays(pScaleAVFrame->data, pScaleAVFrame->linesize, out_buffer, AV_PIX_FMT_YUV420P, dst_w, dst_h, 16);
        LOGE("33b, ret=%d", ret);

        int slice_height =sws_scale(resize, pFrame->data, pFrame->linesize, 0, height, pScaleAVFrame->data, pScaleAVFrame->linesize);

/*
        FILE *file = fopen("/mnt/sdcard/scale.yuv", "wb");
        if (!file){
            LOGE("File open failed");
        }
        size_t y_size = (size_t) (dst_w * dst_h);
        fwrite(pScaleAVFrame->data[0], 1, y_size, file);
        fwrite(pScaleAVFrame->data[1], 1, y_size / 4, file);
        fwrite(pScaleAVFrame->data[2], 1, y_size / 4, file);
        if (file) fclose(file);
*/
//        x264_encode(pScaleAVFrame, dst_w, dst_h);//X264 encoder
        openh264_encode(pScaleAVFrame, dst_w, dst_h);

err_out:
        if (resize) sws_freeContext(resize);
        if (c) {
            avcodec_close(c);
            av_free(c);
        }
        if (pScaleAVFrame) av_frame_free(&pScaleAVFrame);
        LOGE("%s: success", __func__);
    }

    return 0;
}

JNIEXPORT void JNICALL
Java_com_hhbgk_example_androidyuvencoder_CodecWrapper_init(JNIEnv *env, jclass type)
{
    LOGE("%s", __func__);
    //av_log_set_callback(custom_log);
    av_register_all();
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    pCodecCtx = avcodec_alloc_context3(pCodec);
    if(pCodec==NULL){
        LOGE("Couldn't find Codec.\n");
        return ;
    }
    if(avcodec_open2(pCodecCtx, pCodec,NULL)<0){
        LOGE("Couldn't open codec.\n");
        return ;
    }
    pFrame= av_frame_alloc();
    pCodecCtx->flags |= CODEC_FLAG_LOW_DELAY;
    pCodecCtx->debug |= FF_DEBUG_MMCO;
    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;

    pCodecCtx->flags2 |= CODEC_FLAG2_CHUNKS;
    if (pCodec->capabilities & AV_CODEC_CAP_TRUNCATED)
        pCodecCtx->flags |= AV_CODEC_FLAG_TRUNCATED; // we do not send complete frames

    LOGE("%s: success", __func__);
}
JNIEXPORT void JNICALL
Java_com_hhbgk_example_androidyuvencoder_CodecWrapper_destroy(JNIEnv *env, jclass type)
{
    LOGE("%s", __func__);
    av_frame_free(&pFrame);
    avcodec_close(pCodecCtx);
    av_free(pCodecCtx);
}

JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved)
{
    return JNI_VERSION_1_6;
}