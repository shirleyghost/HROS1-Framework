/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <string.h>
#include <string>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("motiondirector", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(0.1);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  int src,slc,srua,slua,srla,slla,srp,slp,srull,slull,srul,slul,srll,slll,srffn,slffn,srar,slar,srn,srs;
  int p,t;
  std::string s;
  std::string sh="";
  src=387;
  slc=641;
  srua=460;
  slua=563;
  srla=452;
  slla=572;
  srp=510;
  slp=510;
  srull=512;
  slull=510;
  srul=494;
  slul=521;
  srll=498;
  slll=514;
  srffn=531;
  slffn=488;
  srar=508;
  slar=507;
  srn=509;
  srs=512;
  p=0;
  t=200;
  int value[22]={src,slc,srua,slua,srla,slla,srp,slp,srull,slull,srul,slul,srll,slll,srffn,slffn,srar,slar,srn,srs,p,t};
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;
    //std_msgs::String s;
    std::stringstream ss;
   int i= rand()%5;
   switch (i)
   {
	case 0:
		ss << "Left" ;
		msg.data = ss.str();
		break;
	case 1:
		ss << "Right";
		msg.data = ss.str();
		break;
	case 2:
		ss << "Walk";
		msg.data = ss.str();
		break;
	case 3:
		ss << "Stopwalk";
		msg.data = ss.str();
		break;
	case 4:
		s="";
	    	s=s+"PLAYPAGE:";
	    	s=s+"|";
	   	for(int i=1;i<21;i++)
	   	{
			ss.str("");
			ss<<i;
			s=s+ss.str();
			ss.str("");	
			ss<<value[i-1];
			s=s+":";
			s=s+ss.str();
			s=s+",";
	   	 }
	   	ss.str("");
	    	ss<<value[20];
	    	s=s+"P:"+ss.str();
	   	s=s+",";
	   	ss.str("");
	   	ss<<value[21];
	  	s=s+"T:"+ss.str();
		s=s+"|";
		msg.data = s;
		break;
    }
/*
    if(i==0)
    {
	ss << "Left" ;
	msg.data = ss.str();
    }
    else 
    {
	if(i==1)
	{
		ss << "Right";
		msg.data = ss.str();
	}
	else
	{
    
//PLAYPOSE:|1:774,2:543,3:456,4:611,5:352,6:417,7:515,8:519,9:516,10:530,11:266,12:760,13:34,14:989,15:746,16:281,17:515,18:528,19:641,20:520,P:0,T:120,|
	    s="";
	    s=s+"PLAYPAGE:";
	    s=s+"|";
	    for(int i=1;i<21;i++)
	    {
		ss.str("");
		ss<<i;
		s=s+ss.str();
		ss.str("");	
		ss<<value[i-1];
		s=s+":";
		s=s+ss.str();
		s=s+",";
	    }
	    ss.str("");
	    ss<<value[20];
	    s=s+"P:"+ss.str();
	    s=s+",";
	    ss.str("");
	    ss<<value[21];
	    s=s+"T:"+ss.str();*/
	    /*s=s+"1:"+boost::lexical_cast<string>(src)+",2:"+boost::lexical_cast<string>(slc)+",3:"+boost::lexical_cast<string>(srua)+",4:"+boost::lexical_cast<string>(slua)+",5:"+boost::lexical_cast<string>(srla)+",6:"+boost::lexical_cast<string>(slla)+",7:"+boost::lexical_cast<string>(srp)+",8:"+boost::lexical_cast<string>(slp)+",9:"+boost::lexical_cast<string>(srull)+",10:"+boost::lexical_cast<string>(slull)+",11:"+boost::lexical_cast<string>(srul)+",12:"+boost::lexical_cast<string>(slul)+",13:"+boost::lexical_cast<string>(srll)+",14:"+boost::lexical_cast<string>(slll)+",15:"+boost::lexical_cast<string>(srffn)+",16:"+boost::lexical_cast<string>(slffn)+",17:"+boost::lexical_cast<string>(srar)+",18:"+boost::lexical_cast<string>(slar)+",19:"+boost::lexical_cast<string>(srn)+",20:"+boost::lexical_cast<string>(srs)+",P:"+boost::lexical_cast<string>(p)+",T:"+boost::lexical_cast<string>(t)",";*/
	  /*  s=s+"|";
    
    //ss << s;
    	    msg.data = s;
	}
    }*/
// %EndTag(FILL_MESSAGE)%
    
// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%
    
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
