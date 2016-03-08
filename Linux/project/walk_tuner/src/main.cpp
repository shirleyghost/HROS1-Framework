#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <signal.h>
#include <libgen.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cmd_process.h"
#include "mjpg_streamer.h"

#define INI_FILE_PATH       "/home/pi/walk_catkin_ws/src/walk_tuner/src/config.ini"

using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Walk tuner: [%s]", msg->data.c_str());
    std::string str = msg->data;
    if (str=="Walk")
    {
        
	LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
        ArbotixPro arbotixpro(&linux_arbotixpro);
	//DrawIntro(&arbotixpro);
        MotionManager::GetInstance()->SetEnable(true);
        MotionManager::GetInstance()->ResetGyroCalibration();
        Walking::GetInstance()->X_OFFSET=12;
	Walking::GetInstance()->Y_OFFSET=20;
	Walking::GetInstance()->Z_OFFSET=20;
	Walking::GetInstance()->R_OFFSET=0.0;
	Walking::GetInstance()->P_OFFSET=-12.0;
	Walking::GetInstance()->A_OFFSET=0.0;
	Walking::GetInstance()->HIP_PITCH_OFFSET=8.0;
	Walking::GetInstance()->BALANCE_ENABLE == true;
	Walking::GetInstance()->PERIOD_TIME=1100;
	Walking::GetInstance()->DSP_RATIO=0.50;
	Walking::GetInstance()->STEP_FB_RATIO=0.30;
	Walking::GetInstance()->X_MOVE_AMPLITUDE=20;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE=0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE=0.0;
	Walking::GetInstance()->A_MOVE_AIM_ON == true;
	Walking::GetInstance()->Z_MOVE_AMPLITUDE=30;
	Walking::GetInstance()->Y_SWAP_AMPLITUDE=25.0;
	Walking::GetInstance()->Z_SWAP_AMPLITUDE=2;
	Walking::GetInstance()->PELVIS_OFFSET=0.1;
	Walking::GetInstance()->ARM_SWING_GAIN=1.8;
	Walking::GetInstance()->BALANCE_KNEE_GAIN=0.07;
	Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN=0.07;
	Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN=0.07;
	Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN=0.07;
	Walking::GetInstance()->BALANCE_ANGLE_GAIN=0.07;
	Walking::GetInstance()->BALANCE_ANGLE_SMOOTH_GAIN=0.95;
	Walking::GetInstance()->LEAN_FB_ACCEL=0.00;
	Walking::GetInstance()->Start();
	ROS_INFO("Start walking");
	
	while(Walking::GetInstance()->IsRunning() == 1)
	{	
		//MotionManager::GetInstance()->Process();
		scheck(arbotixpro);
		
	}
    }
    else if (str=="Stopwalk")
	Walking::GetInstance()->Stop();
}

int main(int argc, char *argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);
    ros::init(argc,argv, "WALK_TUNER");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("motiondirector", 1000, chatterCallback);
    change_current_dir();

    LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
    ArbotixPro arbotixpro(&linux_arbotixpro);
    minIni* ini;
    if (argc == 2)
        ini = new minIni(argv[1]);
    else
        ini = new minIni(INI_FILE_PATH);

    mjpg_streamer* streamer = new mjpg_streamer(0, 0);
    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
    {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
    }
    Walking::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Start();
    /////////////////////////////////////////////////////////////////////
    DrawIntro(&arbotixpro);
    MotionManager::GetInstance()->SetEnable(true);
    MotionManager::GetInstance()->ResetGyroCalibration();
    
    MotionManager::GetInstance()->Process();
    ros::spin();
    exit(0);
    /*
    
	 while(1)
		    {
			if(MotionStatus::FALLEN != STANDUP && (Walking::GetInstance()->IsRunning() == true) )
			 {
			     Walking::GetInstance()->Stop();

			     while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);

			     Action::GetInstance()->m_Joint.SetEnableBody(true, true);

			    if(MotionStatus::FALLEN == FORWARD)
				Action::GetInstance()->Start(1);   // FORWARD GETUP 10
			    else if(MotionStatus::FALLEN == BACKWARD)
				Action::GetInstance()->Start(2);   // BACKWARD GETUP 11
			     while(Action::GetInstance()->IsRunning() == 1) usleep(8000);

			    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			 }
	
		    }
   */
  
   
  /*  while (1)
        {
            int ch = _getch();
            if (ch == 0x1b)
                {
                    ch = _getch();
                    if (ch == 0x5b)
                        {
                            ch = _getch();
                            if (ch == 0x41) // Up arrow key
                                MoveUpCursor();
                            else if (ch == 0x42) // Down arrow key
                                MoveDownCursor();
                            else if (ch == 0x44) // Left arrow key
                                MoveLeftCursor();
                            else if (ch == 0x43)
                                MoveRightCursor();
                        }
                }
            else if ( ch == '[' )
                DecreaseValue(false);
            else if ( ch == ']' )
                IncreaseValue(false);
            else if ( ch == '{' )
                DecreaseValue(true);
            else if ( ch == '}' )
                IncreaseValue(true);
            else if ( ch >= 'A' && ch <= 'z' )
                {
                    char input[128] = {0,};
                    char *token;
                    int input_len;
                    char cmd[80];
                    char strParam[20][30];
                    int num_param;

                    int idx = 0;

                    BeginCommandMode();

                    printf("%c", ch);
                    input[idx++] = (char)ch;

                    while (1)
                        {
                            ch = _getch();
                            if ( ch == 0x0A )
                                break;
                            else if ( ch == 0x7F )
                                {
                                    if (idx > 0)
                                        {
                                            ch = 0x08;
                                            printf("%c", ch);
                                            ch = ' ';
                                            printf("%c", ch);
                                            ch = 0x08;
                                            printf("%c", ch);
                                            input[--idx] = 0;
                                        }
                                }
                            else if ( ch >= 'A' && ch <= 'z' )
                                {
                                    if (idx < 127)
                                        {
                                            printf("%c", ch);
                                            input[idx++] = (char)ch;
                                        }
                                }
                        }

                    fflush(stdin);
                    input_len = strlen(input);
                    if (input_len > 0)
                        {
                            token = strtok( input, " " );
                            if (token != 0)
                                {
                                    strcpy( cmd, token );
                                    token = strtok( 0, " " );
                                    num_param = 0;
                                    while (token != 0)
                                        {
                                            strcpy(strParam[num_param++], token);
                                            token = strtok( 0, " " );
                                        }

                                    if (strcmp(cmd, "exit") == 0)
                                        {
                                            if (AskSave() == false)
                                                break;
                                        }
                                    if (strcmp(cmd, "re") == 0)
                                        DrawScreen();
                                    else if (strcmp(cmd, "save") == 0)
                                        {
                                            Walking::GetInstance()->SaveINISettings(ini);
                                            SaveCmd();
                                        }
                                    else if (strcmp(cmd, "mon") == 0)
                                        {
                                            MonitorCmd();
                                        }
                                    else if (strcmp(cmd, "help") == 0)
                                        HelpCmd();
                                    else
                                        PrintCmd("Bad command! please input 'help'");
                                }
                        }

                    EndCommandMode();
                }
        }

    DrawEnding();*/
    //
    //ros::spin();
   
}
