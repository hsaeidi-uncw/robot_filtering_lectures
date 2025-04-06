#include<ros/ros.h>
#include<turtlesim/Pose.h>


turtlesim::Pose pos_msg;

bool pos_received = false;



void get_pose(const turtlesim::Pose & _data){
	pos_msg = _data; // update based on the most recent reading
	pos_received = true;// flip the flag
}

int main(int argc, char * argv[]){
	// initialize the node
	ros::init(argc, argv, "filter_noise_example");
	// define a node handler
	ros::NodeHandle nh;
	// define a publisher
	ros::Publisher pos_pub =nh.advertise<turtlesim::Pose>("/turtle1/filtered_pose",10);
	// define a subscriber	
	ros::Subscriber pos_sub = nh.subscribe("/turtle1/pose",10, get_pose);
	
	
	int loop_freq = 10; // 10 Hz loop frequency
	ros::Rate loop_rate(loop_freq); // setup the timer interrupt
	
	
	float fil_in = 0.0; // before the first reading
	float fil_out = 5.5; // an initial guess before the first reading
	float fil_gain = 0.05; // how much of the most recent input is included  
	while (ros::ok()){

		fil_in = pos_msg.x;
		std::cout <<"\nfil_in: "<< fil_in<< " fil_out now: "<< fil_out; 
		//filter the position
		fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out;
		std::cout <<", fil_out_next: "<< fil_out; 
		//update and publish the message
		pos_msg.x = fil_out;
		pos_pub.publish(pos_msg);
		//update the topics
		ros::spinOnce();
		//wait for the next iteration
		loop_rate.sleep();

	}
	return 0;
}
