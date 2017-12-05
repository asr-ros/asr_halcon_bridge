/**

Copyright (c) 2016, Allgeyer Tobias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <asr_halcon_bridge/halcon_pointcloud.h>
#include <boost/make_shared.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace halcon_bridge {

    int getSizeFromDatatype(int datatype) {
        if (datatype == sensor_msgs::PointField::INT8) return 1;
        if (datatype == sensor_msgs::PointField::UINT8) return 1;
        if (datatype == sensor_msgs::PointField::INT16) return 2;
        if (datatype == sensor_msgs::PointField::UINT16) return 2;
        if (datatype == sensor_msgs::PointField::INT32) return 4;
        if (datatype == sensor_msgs::PointField::UINT32) return 4;
        if (datatype == sensor_msgs::PointField::FLOAT32) return 4;
        if (datatype == sensor_msgs::PointField::FLOAT64) return 8;

        return -1;
    }

    HalconPointcloud::~HalconPointcloud() {
        delete model;
        curvature.Clear();
    }

    sensor_msgs::PointCloud2Ptr HalconPointcloud::toPointcloudMsg() const {
        sensor_msgs::PointCloud2Ptr ptr = boost::make_shared<sensor_msgs::PointCloud2>();
        toPointcloudMsg(*ptr);
        return ptr;
    }

    void HalconPointcloud::toPointcloudMsg(sensor_msgs::PointCloud2& ros_pointcloud) const {
        ros_pointcloud.header = header;
        ros_pointcloud.height = 1;
        ros_pointcloud.width = (int)model->GetObjectModel3dParams("num_points")[0];
        ros_pointcloud.is_dense = false;
        ros_pointcloud.is_bigendian = false;
        bool has_normals = ((HalconCpp::HString)model->GetObjectModel3dParams("has_point_normals")) == HalconCpp::HString("true");
        std::vector<sensor_msgs::PointField> fields;
        if (has_normals) {
            ros_pointcloud.point_step = 28;
            fields = std::vector<sensor_msgs::PointField>(7);
        } else {
            ros_pointcloud.point_step = 12;
            fields = std::vector<sensor_msgs::PointField>(3);
        }

        ros_pointcloud.row_step = ros_pointcloud.width * ros_pointcloud.point_step;


        sensor_msgs::PointField x_field;
        x_field.name = "x";
        x_field.count = 1;
        x_field.datatype = sensor_msgs::PointField::FLOAT32;
        x_field.offset = 0;
        fields[0] = x_field;

        sensor_msgs::PointField y_field;
        y_field.name = "y";
        y_field.count = 1;
        y_field.datatype = sensor_msgs::PointField::FLOAT32;
        y_field.offset = 4;
        fields[1] = y_field;

        sensor_msgs::PointField z_field;
        z_field.name = "z";
        z_field.count = 1;
        z_field.datatype = sensor_msgs::PointField::FLOAT32;
        z_field.offset = 8;
        fields[2] = z_field;

        if (has_normals) {
            sensor_msgs::PointField x_normals_field;
            x_normals_field.name = "normal_x";
            x_normals_field.count = 1;
            x_normals_field.datatype = sensor_msgs::PointField::FLOAT32;
            x_normals_field.offset = 12;
            fields[3] = x_normals_field;

            sensor_msgs::PointField y_normals_field;
            y_normals_field.name = "normal_y";
            y_normals_field.count = 1;
            y_normals_field.datatype = sensor_msgs::PointField::FLOAT32;
            y_normals_field.offset = 16;
            fields[4] = y_normals_field;

            sensor_msgs::PointField z_normals_field;
            z_normals_field.name = "normal_z";
            z_normals_field.count = 1;
            z_normals_field.datatype = sensor_msgs::PointField::FLOAT32;
            z_normals_field.offset = 20;
            fields[5] = z_normals_field;

            sensor_msgs::PointField curvature;
            curvature.name = "curvature";
            curvature.count = 1;
            curvature.datatype = sensor_msgs::PointField::FLOAT32;
            curvature.offset = 24;
            fields[6] = curvature;
        }


        ros_pointcloud.fields = fields;


        HalconCpp::HTuple x_values = model->GetObjectModel3dParams("point_coord_x");
        HalconCpp::HTuple y_values = model->GetObjectModel3dParams("point_coord_y");
        HalconCpp::HTuple z_values = model->GetObjectModel3dParams("point_coord_z");

        HalconCpp::HTuple x_normal_values;
        HalconCpp::HTuple y_normal_values;
        HalconCpp::HTuple z_normal_values;


        float *interleavedPoints;

        if (has_normals) {
            x_normal_values = model->GetObjectModel3dParams("point_normal_x");
            y_normal_values = model->GetObjectModel3dParams("point_normal_y");
            z_normal_values = model->GetObjectModel3dParams("point_normal_z");
            interleavedPoints = new float[ros_pointcloud.width * 7];
        } else {
            interleavedPoints = new float[ros_pointcloud.width * 3];
        }

        for (size_t i = 0; i < ros_pointcloud.width; i++) {
            interleavedPoints[i * 3] = (float)x_values[i];
            interleavedPoints[i * 3 + 1] = (float)y_values[i];
            interleavedPoints[i * 3 + 2] = (float)z_values[i];
            if (has_normals) {
                interleavedPoints[i * 3 + 3] = (float)x_normal_values[i];
                interleavedPoints[i * 3 + 4] = (float)y_normal_values[i];
                interleavedPoints[i * 3 + 5] = (float)z_normal_values[i];
                interleavedPoints[i * 3 + 6] = (float)(curvature)[i];
            }
        }

        size_t size;
        if (has_normals) {
            size = ros_pointcloud.width * 7 * 4;
        } else {
            size = ros_pointcloud.width * 3 * 4;
        }
        ros_pointcloud.data.resize(size);

        memcpy((float*)(&ros_pointcloud.data[0]), &interleavedPoints[0], size);


    }



    HalconPointcloudPtr toHalconCopy(const sensor_msgs::PointCloud2ConstPtr& source) {
         return toHalconCopy(*source);
    }

    HalconPointcloudPtr toHalconCopy(const sensor_msgs::PointCloud2& source) {
        HalconPointcloudPtr ptr = boost::make_shared<HalconPointcloud>();
        ptr->header = source.header;


        int offset_x, offset_y, offset_z, offset_x_normal, offset_y_normal, offset_z_normal, offset_curvature;
        offset_x = offset_y = offset_z = offset_x_normal = offset_y_normal = offset_z_normal = offset_curvature = 0;
        int count_x_normal, count_y_normal, count_z_normal, count_curvature;
        count_x_normal = count_y_normal = count_z_normal = count_curvature = 0;

        for (unsigned int i = 0; i < source.fields.size(); i++) {
            sensor_msgs::PointField field = source.fields[i];
            if (field.name == "x") {
                offset_x = field.offset;
            }
            if (field.name == "y") {
                offset_y = field.offset;
            }
            if (field.name == "z") {
                offset_z = field.offset;
            }
            if (field.name == "normal_x") {
                offset_x_normal = field.offset;
                count_x_normal = field.count;
            }
            if (field.name == "normal_y") {
                offset_y_normal = field.offset;
                count_y_normal = field.count;
            }
            if (field.name == "normal_z") {
                offset_z_normal = field.offset;
                count_z_normal = field.count;
            }
            if (field.name == "curvature") {
                offset_curvature = field.offset;
                count_curvature = field.count;
            }

        }

        HalconCpp::HTuple x_coords = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);
        HalconCpp::HTuple y_coords = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);
        HalconCpp::HTuple z_coords = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);

        HalconCpp::HTuple x_normals = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);
        HalconCpp::HTuple y_normals = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);
        HalconCpp::HTuple z_normals = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);

        HalconCpp::HTuple curvature = HalconCpp::HTuple::TupleGenConst((int)(source.width * source.height), 0);

        for (int i = 0; i < x_coords.Length(); i++) {
            x_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_x];
            y_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_y];
            z_coords[i] = *(float*)&source.data[(i * source.point_step) + offset_z];

            if ((count_x_normal > 0) && (count_y_normal > 0) && (count_z_normal > 0)) {
                x_normals[i] = *(float*)&source.data[(i * source.point_step) + offset_x_normal];
                y_normals[i] = *(float*)&source.data[(i * source.point_step) + offset_y_normal];
                z_normals[i] = *(float*)&source.data[(i * source.point_step) + offset_z_normal];
            }

        }
        ptr->model = new HalconCpp::HObjectModel3D(x_coords, y_coords, z_coords);

        if (source.fields.size() > 4) {
            HalconCpp::HTuple attrib_names("point_normal_x");
            attrib_names.Append("point_normal_y");
            attrib_names.Append("point_normal_z");
            HalconCpp::HTuple attrib_values = (x_normals.TupleConcat(y_normals)).TupleConcat(z_normals);
            HalconCpp::HObjectModel3D p_n_model = ptr->model->SetObjectModel3dAttrib(attrib_names, "", attrib_values);
            *ptr->model = p_n_model;

            for (int i = 0; i < curvature.Length(); i++) {
                curvature[i] = *(float*)&source.data[(i * source.point_step) + offset_curvature];
            }
            ptr->curvature = curvature;
        }

        return ptr;
    }

}



