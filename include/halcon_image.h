/**

Copyright (c) 2016, Allgeyer Tobias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <sensor_msgs/Image.h>
#include <HalconCpp.h>
#include <stdexcept>

namespace halcon_bridge {

    class Exception: public std::runtime_error {
        public:
            Exception(const std::string& description) :
                std::runtime_error(description) {
            }
    };


    class HalconImage;

    typedef boost::shared_ptr<HalconImage> HalconImagePtr;

    /**
     * \brief Image message class that is interoperable with sensor_msgs/Image but uses a HImage representation for the image data.
     *
     * @author Allgeyer Tobias
     */
    class HalconImage {
        public:
            std_msgs::Header header;
            std::string encoding;
            HalconCpp::HImage *image;

            ~HalconImage();

            /**
             * \brief Convert this message to a ROS sensor_msgs::Image message.
             *
             * The returned sensor_msgs::Image message contains a copy of the image data.
             */
            sensor_msgs::ImagePtr toImageMsg() const;

            /**
             * \brief Copy the message data to a ROS sensor_msgs::Image message.
             *
             * This overload is intended mainly for aggregate messages such as stereo_msgs::DisparityImage,
             * which contains a sensor_msgs::Image as a data member.
             */
            void toImageMsg(sensor_msgs::Image& ros_image) const;
    };


    /**
     * \brief Convert a sensor_msgs::Image message to a Halcon-compatible HImage, copying the
     * image data.
     *
     * \param source   A shared_ptr to a sensor_msgs::Image message
     */
    HalconImagePtr toHalconCopy(const sensor_msgs::ImageConstPtr& source);

    /**
     * \brief Convert a sensor_msgs::Image message to a Halcon-compatible HImage, copying the
     * image data.
     *
     * \param source   A sensor_msgs::Image message
     */
    HalconImagePtr toHalconCopy(const sensor_msgs::Image& source);



}
