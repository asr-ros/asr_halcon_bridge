/**

Copyright (c) 2016, Allgeyer Tobias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <sensor_msgs/PointCloud2.h>
#include <HalconCpp.h>
#include <stdexcept>

namespace halcon_bridge {




    class HalconPointcloud;

    typedef boost::shared_ptr<HalconPointcloud> HalconPointcloudPtr;

    /**
     * \brief PointCloud message class that is interoperable with sensor_msgs/PointCloud2 but uses a HObjectModel3D representation for the point cloud data.
     *
     * @author Allgeyer Tobias
     */
    class HalconPointcloud {
        public:
            std_msgs::Header header;
            HalconCpp::HTuple curvature;
            HalconCpp::HObjectModel3D *model;

            ~HalconPointcloud();

            /**
             * \brief Convert this message to a ROS sensor_msgs::PointCloud2 message.
             *
             * The returned sensor_msgs::PointCloud2 message contains a copy of the Halcon-ObjectModel data.
             */
            sensor_msgs::PointCloud2Ptr toPointcloudMsg() const;


            /**
             * \brief Copy the message data to a ROS sensor_msgs::PointCloud2 message.
             *
             */
            void toPointcloudMsg(sensor_msgs::PointCloud2& ros_pointcloud) const;
    };


    /**
     * \brief Convert a sensor_msgs::PointCloud2 message to a Halcon-compatible HObjectModel3D, copying the
     * point cloud data.
     *
     * \param source   A shared_ptr to a sensor_msgs::PointCloud2 message
     *
     */
    HalconPointcloudPtr toHalconCopy(const sensor_msgs::PointCloud2ConstPtr& source);


    /**
     * \brief Convert a sensor_msgs::PointCloud2 message to a Halcon-compatible HObjectModel3D, copying the
     * point cloud data.
     *
     * \param source   A sensor_msgs::PointCloud2 message
     *
     */
    HalconPointcloudPtr toHalconCopy(const sensor_msgs::PointCloud2& source);



}

