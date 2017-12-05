/**

Copyright (c) 2016, Allgeyer Tobias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <asr_halcon_bridge/halcon_image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/make_shared.hpp>

namespace halcon_bridge {

    const char* INVALID = "invalid";
    const char* RGB = "rgb";
    const char* BGR = "bgr";

    int getHalconTypeSize(const std::string& type) {

        if ((type == "byte") || (type == "int1")) return 1;
        if ((type == "uint2") || (type == "int2")) return 2;
        if ((type == "int4") || (type == "real")) return 4;
        if (type == "int8") return 8;

        return -1;
    }

    const char* getHalconEncoding(const std::string& encoding) {
      // 3/4-channel encodings
      if (encoding == sensor_msgs::image_encodings::BGR8)   return "bgr";
      if (encoding == sensor_msgs::image_encodings::RGB8)   return "rgb";
      if (encoding == sensor_msgs::image_encodings::BGRA8)  return "bgrx";
      if (encoding == sensor_msgs::image_encodings::RGBA8)  return "rgbx";

      // 1-channel encoding
      if (encoding == sensor_msgs::image_encodings::MONO8)  return "mono";

      // Other formats are not supported
      return INVALID;
    }



    const char* getHalconChannelLength(const std::string& encoding) {

        if ((encoding == sensor_msgs::image_encodings::BGR8) || (encoding == sensor_msgs::image_encodings::RGB8) ||
                (encoding == sensor_msgs::image_encodings::BGRA8) || (encoding == sensor_msgs::image_encodings::RGBA8) ||
                (encoding == sensor_msgs::image_encodings::MONO8)) {
          return "byte";
        }
      if ((encoding == sensor_msgs::image_encodings::BGR16) || (encoding == sensor_msgs::image_encodings::RGB16) ||
              (encoding == sensor_msgs::image_encodings::BGRA16) || (encoding == sensor_msgs::image_encodings::RGBA16) ||
              (encoding == sensor_msgs::image_encodings::MONO16)) {
         return "uint2";
      }

      return INVALID;
    }



    const char* getColorChannelOrder(const std::string& encoding) {
        if ((encoding == sensor_msgs::image_encodings::BGR8) || (encoding == sensor_msgs::image_encodings::BGRA8) ||
                (encoding == sensor_msgs::image_encodings::BGR16) || (encoding == sensor_msgs::image_encodings::BGRA16)) {
            return BGR;
        }
        if ((encoding == sensor_msgs::image_encodings::RGB8) || (encoding == sensor_msgs::image_encodings::RGBA8) ||
                (encoding == sensor_msgs::image_encodings::RGB16) || (encoding == sensor_msgs::image_encodings::RGBA16)) {
            return RGB;
        }
        return INVALID;
    }




    HalconImage::~HalconImage() {
        delete image;
    }

    sensor_msgs::ImagePtr HalconImage::toImageMsg() const {
      sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
      toImageMsg(*ptr);
      return ptr;
    }



    void HalconImage::toImageMsg(sensor_msgs::Image& ros_image) const {
        long width, height;
        width = image->Width();
        height = image->Height();

        int channel_count = image->CountChannels();

        ros_image.height = height;
        ros_image.width = width;
        ros_image.encoding = encoding;
        ros_image.is_bigendian = false;
        ros_image.step = channel_count * width;


        HalconCpp::HString typeReturn;
        Hlong widthReturn;
        Hlong heightReturn;
        HalconCpp::HString type = image->GetImageType();

        if (channel_count > 1) {

            //build interleaved image from 3-channel image

            HalconCpp::HImage *interleavedImage = new HalconCpp::HImage(type, width * 3, height);

            HalconCpp::HHomMat2D homMat;
            homMat = homMat.HomMat2dScale(1, 3, 0, 0);

            HalconCpp::HImage transImage = image->AffineTransImageSize(homMat, "constant", width * 3, height);

            HalconCpp::HImage imageRed;
            HalconCpp::HImage imageGreen;
            HalconCpp::HImage imageBlue;
            imageRed = transImage.Decompose3(&imageGreen, &imageBlue);


            HalconCpp::HRegion regionGrid;
            regionGrid.GenGridRegion(2 * height, 3, "lines", width * 3, height + 1);
            HalconCpp::HRegion movedRegion = regionGrid.MoveRegion(-1, 0);
            HalconCpp::HRegion clippedRegion = movedRegion.ClipRegion(0, 0, height - 1, (3 * width) - 1);

            if (getColorChannelOrder(encoding) == RGB) {
                imageRed = imageRed.ReduceDomain(clippedRegion);
            } else {
                imageBlue = imageBlue.ReduceDomain(clippedRegion);
            }
            movedRegion = regionGrid.MoveRegion(-1, 1);
            clippedRegion = movedRegion.ClipRegion(0, 0, height - 1, (3 * width) - 1);
            imageGreen = imageGreen.ReduceDomain(clippedRegion);
            movedRegion = regionGrid.MoveRegion(-1, 2);
            clippedRegion = movedRegion.ClipRegion(0, 0, height - 1, (3 * width) - 1);
            if (getColorChannelOrder(encoding) == RGB) {
                imageBlue = imageBlue.ReduceDomain(clippedRegion);
            } else {
                imageRed = imageRed.ReduceDomain(clippedRegion);
            }

            interleavedImage->OverpaintGray(imageRed);
            interleavedImage->OverpaintGray(imageGreen);
            interleavedImage->OverpaintGray(imageBlue);



            // copy data of interleaved image into sensor_msg
            unsigned char* colorData = (unsigned char*)interleavedImage->GetImagePointer1(&typeReturn, &widthReturn, &heightReturn);
            int interleavedHeight = interleavedImage->Height();
            int interleavedWidth =  interleavedImage->Width();
            size_t size = interleavedHeight * interleavedWidth * getHalconTypeSize((std::string)typeReturn);
            ros_image.data.resize(size);
            memcpy((unsigned char*)(&ros_image.data[0]), colorData, size);

            interleavedImage->Clear();
            imageRed.Clear();
            imageGreen.Clear();
            imageBlue.Clear();
            homMat.Clear();
            regionGrid.Clear();
            clippedRegion.Clear();
            movedRegion.Clear();


        } else {

            // 1-channel image: copy data of original image
            unsigned char* colorData = (unsigned char*)image->GetImagePointer1(&typeReturn, &widthReturn, &heightReturn);
            size_t size = height * width * getHalconTypeSize((std::string)typeReturn);
            ros_image.data.resize(size);
            memcpy((unsigned char*)(&ros_image.data[0]), colorData, size);
        }

    }



    HalconImagePtr toHalconCopy(const sensor_msgs::ImageConstPtr& source) {
      return toHalconCopy(*source);
    }

    HalconImagePtr toHalconCopy(const sensor_msgs::Image& source) {
        HalconImagePtr ptr = boost::make_shared<HalconImage>();
        ptr->header = source.header;
        ptr->encoding = source.encoding;

        if (getHalconEncoding(ptr->encoding) == INVALID) {
            throw Exception("Encoding " + ptr->encoding + " not supported");
        }

        long* pixeldata = (long*)const_cast<unsigned char*>(&source.data[0]);
        HalconCpp::HImage *img = new HalconCpp::HImage();
        if ((std::string)getHalconEncoding(source.encoding) == "mono") {
             img->GenImage1(getHalconChannelLength(source.encoding), source.width, source.height, pixeldata);
        } else {
            img->GenImageInterleaved(pixeldata, getHalconEncoding(source.encoding), source.width, source.height, 0,
                                 getHalconChannelLength(source.encoding), source.width, source.height, 0, 0, -1, 0);
        }
        ptr->image = img;

        return ptr;
    }

}


