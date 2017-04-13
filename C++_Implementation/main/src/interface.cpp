/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \interface.cpp
///
/// \Definition : This is interface function for H2 and APP
///
///  \author XipengCui
///  \Date : Created on: Mar 1, 2017
///
#include "../edge/include/edge_boxes_interface.h"
#include "../tools/include/data_define.h"
#include "../tools/include/image_tools.h"

//#define IS_SAVE
//extern "C" void InitializedBox(char* model_file);
//extern "C" int GetBox(unsigned char* buffer, int width, int height, float x_, float y_, float w_, float h_, int data_format, float* _x, float* _y, float* _width, float* _height);

void ResizeROI(unsigned char* src, unsigned char* dst, autel::computer_vision::AuRect<int> roi, autel::computer_vision::AuSize src_size, int src_step, int cn, autel::computer_vision::AuSize dst_size, int dst_step);

void InitializedBox(char* model_file)
{
  if(model_file == NULL)
  {
    char model_path[256] = "/tmp/SD0/model_train.txt";
    InitEdgeBox(model_path);
  }
  else
    InitEdgeBox(model_file);
}

void ResizeROI(unsigned char* src, unsigned char* dst, autel::computer_vision::AuRect<int> roi, autel::computer_vision::AuSize src_size, int src_step, int cn, autel::computer_vision::AuSize dst_size, int dst_step)
{
    //bilinearity resample
    int w = dst_size.width_;
    int h = dst_size.height_;

    //float --> int
    int s = 10;
    int s_sum = 1<<s;

    int x, y;
    for(y = 0; y < h; y++)
    {
        long test = (long)(y*src_size.height_)<<s;
        long y1 = test/h;
        test = y1>>s;
        int a = y1 - (test<<s);
        y1 = y1>>s;
        y1 += roi.y_;
        for(x = 0; x < w; x++)
        {
            test = (long)(x*src_size.width_)<<s;
            long x1 = test/w;
            test = x1>>s;
            int b = x1 - (test<<s);
            x1 = x1>>s;
            x1 += roi.x_;
            {
                int dst_id = (y*dst_step+x)*cn;
                long data_1 = src[(y1*src_step+x1)*cn];
                long data_2 = src[(y1*src_step+x1+1)*cn];
                long data_3 = src[((y1+1)*src_step+x1+1)*cn];
                long data_4 = src[((y1+1)*src_step+x1)*cn];
                dst[dst_id] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                if(cn > 1)
                {
                    data_1 = src[(y1*src_step+x1)*3+1];
                    data_2 = src[(y1*src_step+x1+1)*3+1];
                    data_3 = src[((y1+1)*src_step+x1+1)*3+1];
                    data_4 = src[((y1+1)*src_step+x1)*3+1];
                    dst[dst_id+1] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));

                    data_1 = src[(y1*src_step+x1)*3+2];
                    data_2 = src[(y1*src_step+x1+1)*3+2];
                    data_3 = src[((y1+1)*src_step+x1+1)*3+2];
                    data_4 = src[((y1+1)*src_step+x1)*3+2];
                    dst[dst_id+2] =(unsigned char)((data_1*a*b + data_2*a*(s_sum-b)+data_3*(s_sum-a)*(s_sum-b) + data_4*(s_sum-a)*b)>>(s+s));
                }
            }
        }
    }
}



