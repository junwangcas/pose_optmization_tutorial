#include "land_mark_vis.h"

LandMarkVis::LandMarkVis(float r, float g, float b, float a)
        : m_marker_ns("PtVis") {
}

void LandMarkVis::reset() {
    ptsMsg_.points.clear();
}

void LandMarkVis::publish_by( ros::Publisher &pub, const std_msgs::Header &header ) {
    ptsMsg_.header = header;
    pub.publish(ptsMsg_);
}

void LandMarkVis::AddPoints(const std::vector<Eigen::Vector3d> &pts)
{
    Eigen2PtsCloud(pts, ptsMsg_);
}

void LandMarkVis::Eigen2PtsCloud(const std::vector<Eigen::Vector3d> &pts, sensor_msgs::PointCloud &ptsMsg)
{
    for (auto &pt : pts) {
        geometry_msgs::Point32 ptros;
        ptros.x = pt.x();
        ptros.y = pt.y();
        ptros.z = pt.z();
        ptsMsg.points.emplace_back(ptros);
    }
}