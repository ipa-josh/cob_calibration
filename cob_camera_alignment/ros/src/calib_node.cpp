/*
 * calib_node.cpp
 *
 *  Created on: 05.12.2012
 *      Author: josh
 */



#include <ros/ros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <tf/transform_listener.h>
#include <XmlRpcException.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_camera_alignment/StartMeasurementsAction.h>

#include <cob_3d_mapping_msgs/ShapeArray.h>
#include <cob_3d_shapes/conversion.h>


class As_Node
{
protected:
  ros::NodeHandle n_;
  tf::TransformListener tf_listener_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  pcl_ros::PCLNodelet
{
protected:
  ros::NodeHandle n_;
public:
  As_Nodelet() {
  }

  virtual ~As_Nodelet() {}

  void start() {
    PCLNodelet::onInit();
    n_ = getNodeHandle();
  }
};

template <typename Parent>
class Calib_Node : public Parent
{

  typedef actionlib::SimpleActionServer<cob_camera_alignment::StartMeasurementsAction> ActionServer;

  ros::Subscriber shapes_sub_;

  static bool sort_shapes_by_weight(const cob_3d_mapping_msgs::Shape &a, const cob_3d_mapping_msgs::Shape &b) {
    return a.weight > b.weight;
  }

  Eigen::Matrix3f getAlignment(const Eigen::Matrix3f &expection, const boost::shared_ptr<cob_3d_shapes::Surface> &a, const boost::shared_ptr<cob_3d_shapes::Surface> &b, const ros::Time &time) {
    ROS_ASSERT_MSG(a, "cannot read this shape type");
    ROS_ASSERT_MSG(b, "cannot read this shape type");

    //std::cout<<a->normalUV(Eigen::Vector2f::Zero())<<"\n";

    Eigen::Vector3f n1 = a->normalNormalized(Eigen::Vector2f::Zero());
    Eigen::Vector3f n2 = b->normalNormalized(Eigen::Vector2f::Zero());

    Eigen::Matrix3f R, Rbest, Rtmp;
    R.col(0) = n1;
    R.col(1) = n2;
    R.col(2) = R.col(0).cross(R.col(1));

    Eigen::Vector3f x,y;
    x.fill(0);y=x;
    x(0)=1;y(1)=1;

    float qual_best = std::numeric_limits<float>::max();
    for(int i=0; i<4; i++)
      for(int j=0; j<4; j++)
      {
        Eigen::AngleAxisf aa1(M_PI/2*i, x), aa2(M_PI/2*j, y);
        Rtmp = aa1*aa2*R;
        float qual = (expection * Rtmp.transpose()).diagonal().squaredNorm();
        if(qual<qual_best) {
          qual_best = qual;
          Rbest = Rtmp;
        }
      }

    return Rbest;
  }

  float getAlignment2(const boost::shared_ptr<cob_3d_shapes::Surface> &a, const Eigen::Vector3f &axis) {
//    std::cout<<a->normalNormalized(Eigen::Vector2f::Zero())<<"\n";
//    std::cout<<axis<<"\n\n";
    ROS_ASSERT_MSG( std::abs(1-axis.squaredNorm())<0.001f, "axis have to be normalized" );
    float d = axis.dot( a->normalNormalized(Eigen::Vector2f::Zero()) );
    if(d<0) return -std::acos( -d );
    return std::acos( d );
  }

  struct AVERAGE {
    float delta_;
    int num_;
    Eigen::Matrix3f cor_;

    AVERAGE(){clear();}

    void operator+=(const float d) {
      ++num_;
      delta_+=d;
    }

    void clear() {
      delta_=0;
      num_=0;
      cor_ = Eigen::Matrix3f::Identity();
    }

    operator float() {
      return delta_/num_;
    }
  };
  std::vector<Eigen::Vector3f> normals_, axis_;
  std::vector<AVERAGE> delta_;
  std::string tf_, world_;
  float threshold_;
  ActionServer as_;
  cob_camera_alignment::StartMeasurementsFeedback feedback_;
  cob_camera_alignment::StartMeasurementsResult result_;
  int needed_;

public:
  Calib_Node():
    as_(this->n_, "/start_measurements", false), needed_(0)
  {}

  void onInit()
  {
    threshold_ = 0.1f;

    ros::NodeHandle nh_("~");

    if (nh_.hasParam("walls"))
    {
      try {
        XmlRpc::XmlRpcValue v;
        nh_.param("walls", v, v);
        for(int i =0; i < v.size(); i++)
        {
          ROS_ASSERT(v[i].size()==3);
          Eigen::Vector3f n;

          n(0) = (double)v[i][0];
          n(1) = (double)v[i][1];
          n(2) = (double)v[i][2];

          n.normalize();

          normals_.push_back(n);
          delta_.push_back(AVERAGE());
        }
      }
      catch(XmlRpc::XmlRpcException &e) {
        ROS_ERROR("error parsing yaml for walls\n%s\nshutdown...", e.getMessage().c_str());
        nh_.shutdown();
        return;
      }
      catch(...) {
        ROS_ERROR("error parsing yaml for walls, shutdown...");
        nh_.shutdown();
        return;
      }
    }
    else
    {
      ROS_ERROR("Parameter walls not set, shutting down node...");
      nh_.shutdown();
      return;
    }


    if (nh_.hasParam("axis"))
    {
      try {
        XmlRpc::XmlRpcValue v;
        nh_.param("axis", v, v);
        for(int i =0; i < v.size(); i++)
        {
          ROS_ASSERT(v[i].size()==3);
          Eigen::Vector3f n;

          n(0) = (double)v[i][0];
          n(1) = (double)v[i][1];
          n(2) = (double)v[i][2];

          n.normalize();

          axis_.push_back(n);
        }
      }
      catch(XmlRpc::XmlRpcException &e) {
        ROS_ERROR("error parsing yaml for walls\n%s\nshutdown...", e.getMessage().c_str());
        nh_.shutdown();
        return;
      }
      catch(...) {
        ROS_ERROR("error parsing yaml for walls, shutdown...");
        nh_.shutdown();
        return;
      }
    }

    if (nh_.hasParam("tf"))
    {
      nh_.getParam("tf",tf_);
    }
    else
    {
      ROS_ERROR("Parameter tf not set, shutting down node...");
      nh_.shutdown();
      return;
    }

    if (nh_.hasParam("world"))
    {
      nh_.getParam("world",world_);
    }
    else
    {
      ROS_ERROR("Parameter world not set, shutting down node...");
      nh_.shutdown();
      return;
    }

    if (nh_.hasParam("threshold"))
    {
      double t;
      nh_.getParam("threshold",t);
      threshold_ = (float)t;
    }

    shapes_sub_ = this->n_.subscribe("/shapes_array", 1, &Calib_Node<Parent>::cbShapes, this);

    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&Calib_Node<Parent>::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Calib_Node<Parent>::preemptCB, this));
    as_.start();
  }

  void goalCB()
  {
    ROS_ASSERT_MSG( as_.acceptNewGoal()->deltas.size()==0 || as_.acceptNewGoal()->deltas.size()==axis_.size(), "axis has to be a 3-D vector for each normal");
    ROS_ASSERT_MSG( as_.acceptNewGoal()->deltas.size()==axis_.size(), "deltas have to be set for each axis");
    for(size_t i=0; i<delta_.size(); i++) {
      delta_[i].clear();
      if(as_.acceptNewGoal()->deltas.size()!=0) {
        Eigen::AngleAxisf aa(as_.acceptNewGoal()->deltas[i], axis_[i].cross(normals_[i]));
        delta_[i].cor_ = aa.toRotationMatrix();
      }
    }
    needed_ = as_.acceptNewGoal()->number_of_frames;
  }

  void preemptCB()
  {
    needed_ = 0;
  }

  void
  cbShapes(cob_3d_mapping_msgs::ShapeArray::ConstPtr cpa)
  {
    if(cpa->shapes.size()<1 || needed_<1) return;

    std::vector<cob_3d_mapping_msgs::Shape> cp = cpa->shapes;
    std::sort(cp.begin(), cp.end(), sort_shapes_by_weight);

    tf::StampedTransform transform;
    try{
      this->tf_listener_.lookupTransform(world_, tf_,
                                         cpa->header.stamp, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    Eigen::Quaternionf q;
    q.x() = transform.getRotation().getX();
    q.y() = transform.getRotation().getY();
    q.z() = transform.getRotation().getZ();
    q.w() = transform.getRotation().getW();

    //std::cout<<q.toRotationMatrix()<<"\n";
    int n=0;
    for(size_t i=0; i<normals_.size(); i++) {

      for(size_t j=0; j<cp.size(); j++) {
        const float d = getAlignment2(cob_3d_shapes::SurfaceFromShapeMsg(cp[j]), delta_[i].cor_*q*normals_[i]);
        if(std::abs(d)>threshold_) {
//          ROS_WARN("delta angle between axis %d is %f",(int)i,d );
          continue;
        }

        ROS_INFO("delta angle between axis %d is %f (%f)",(int)i,d, (float)delta_[i] );
        delta_[i] += d;
        ++n;
        break;
      }
    }

    if(n==(int)normals_.size()) {
      feedback_.number_of_measurements = --needed_;
      as_.publishFeedback(feedback_);
      if(needed_==0) {
        result_.deltas.clear();
        for(size_t i=0; i<delta_.size(); i++)
          result_.deltas.push_back( (float)delta_[i] );
        as_.setSucceeded(result_);
      }
    }

    /*Eigen::Matrix3f R = getAlignment(Eigen::Matrix3f::Identity(), cob_3d_shapes::SurfaceFromShapeMsg(cp[0]), cob_3d_shapes::SurfaceFromShapeMsg(cp[1]), cpa->header.stamp);

    std::cout<<R<<"\n";

    for(size_t i=0; i<axis_.size(); i++) {
      const float d = std::acos( (R*axis_[i]).dot(axis_[i]) );
      ROS_INFO("delta angle between axis %d is %f",(int)i,d );
      delta_[i] += d;
    }*/
  }

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_alignment");

  Calib_Node<As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}
