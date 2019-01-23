#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

trajectory_msgs::MultiDOFJointTrajectory tj;
void CallBack(const trajectory_msgs::MultiDOFJointTrajectory& tj1){
    tj=tj1;
    int i,j;
    for(j=0;j<tj1.points.size();j++){
    for(i=0;i<tj1.points[j].transforms.size();i++){
        tj.points[j].transforms[i].translation.y=(-1)*tj1.points[j].transforms[i].translation.y;
        tj.points[j].transforms[i].translation.x=(-1)*tj1.points[j].transforms[i].translation.x;
    }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sub_boi");
    ros::NodeHandle nh;
    ros::Subscriber sub= nh.subscribe("/firefly1/command/trajectory",1,CallBack);
    ros::Publisher pub=nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly2/command/trajectory",1);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        pub.publish(tj);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}